// Realtime closed-loop stepper correction using SPI magnetic angle sensors
//
// Copyright (C) 2026  Adam Barnas
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// This runs a small integer "accumulate and release" corrector: every tick
// it compares how far the stepper's own commanded position moved against
// how far the angle sensor says the axis actually moved, accumulates the
// (signed) difference in a Q16.16 fixed-point register, and - whenever the
// accumulated error exceeds a configured deadband and the rate limiter
// allows it - injects a single out-of-band correction step via
// stepper_apply_correction_step() (see stepper.c) in whichever direction
// closes the gap.
//
// Scope note: this works on the RAW (uncalibrated, unlinearized) angle
// sample from sensor_angle.c, not the host's calibrated/direction-corrected
// accumulated angle (klippy/extras/angle.py) - replicating that calibration
// table on an FPU-less Cortex-M0+ inside a realtime loop is out of scope.
// The signed "ratio" gain uploaded at config time is expected to absorb the
// axis's direction sense; this is a coarse slip corrector, not a precision
// position source.

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_angle.h" // spi_angle_get_latest
#include "stepper.h" // stepper_get_position

struct closed_loop_stepper {
    struct timer timer;
    uint32_t rest_ticks;
    struct stepper *stepper;
    struct spi_angle *angle;

    // Config (uploaded once at config_closed_loop_stepper time)
    int32_t ratio_q16;          // signed Q16.16 steps-per-raw-angle-count
    uint32_t deadband_q16;      // Q16.16 steps - accumulated error threshold
    uint32_t min_interval_ticks; // rate limiter: min ticks between corrections

    // Live tracking state (reset whenever (re)started via the query command)
    uint32_t prev_angle_raw;
    uint32_t prev_stepper_pos;
    int32_t error_accum_q16;
    uint32_t last_correction_time;
    int32_t corrected_steps;    // cumulative signed correction step count
    uint8_t flags, have_prev;
};

enum {
    CLS_PENDING = 1<<0,
};

static struct task_wake closed_loop_wake;

// Event handler that wakes closed_loop_stepper_task() periodically
static uint_fast8_t
closed_loop_stepper_event(struct timer *timer)
{
    struct closed_loop_stepper *cls = container_of(
        timer, struct closed_loop_stepper, timer);
    cls->flags |= CLS_PENDING;
    sched_wake_task(&closed_loop_wake);
    cls->timer.waketime += cls->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_closed_loop_stepper(uint32_t *args)
{
    struct closed_loop_stepper *cls = oid_alloc(
        args[0], command_config_closed_loop_stepper, sizeof(*cls));
    cls->timer.func = closed_loop_stepper_event;
    cls->stepper = stepper_oid_lookup(args[1]);
    cls->angle = spi_angle_oid_lookup(args[2]);
    cls->ratio_q16 = args[3];
    cls->deadband_q16 = args[4];
    cls->min_interval_ticks = args[5];
}
DECL_COMMAND(command_config_closed_loop_stepper,
             "config_closed_loop_stepper oid=%c stepper_oid=%c angle_oid=%c"
             " ratio=%i deadband=%u min_interval_ticks=%u");

// Start/stop periodic correction.  rest_ticks==0 stops and clears state.
void
command_query_closed_loop_stepper(uint32_t *args)
{
    uint8_t oid = args[0];
    struct closed_loop_stepper *cls = oid_lookup(
        oid, command_config_closed_loop_stepper);

    sched_del_timer(&cls->timer);
    cls->flags = 0;
    cls->have_prev = 0;
    cls->error_accum_q16 = 0;
    cls->last_correction_time = 0;
    cls->corrected_steps = 0;
    if (!args[2])
        // End correction
        return;

    cls->timer.waketime = args[1];
    cls->rest_ticks = args[2];
    sched_add_timer(&cls->timer);
}
DECL_COMMAND(command_query_closed_loop_stepper,
             "query_closed_loop_stepper oid=%c clock=%u rest_ticks=%u");

void
command_closed_loop_stepper_get_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct closed_loop_stepper *cls = oid_lookup(
        oid, command_config_closed_loop_stepper);
    sendf("closed_loop_stepper_state oid=%c corrected_steps=%i error_q16=%i",
          oid, cls->corrected_steps, cls->error_accum_q16);
}
DECL_COMMAND(command_closed_loop_stepper_get_state,
             "closed_loop_stepper_get_state oid=%c");

// Run one correction cycle for a single stepper/angle pair
static void
closed_loop_stepper_check(struct closed_loop_stepper *cls)
{
    uint32_t angle_time, angle_raw;
    if (!spi_angle_get_latest(cls->angle, &angle_time, &angle_raw))
        return;

    irq_disable();
    uint32_t stepper_pos = stepper_get_position(cls->stepper);
    irq_enable();

    if (!cls->have_prev) {
        cls->prev_angle_raw = angle_raw;
        cls->prev_stepper_pos = stepper_pos;
        cls->have_prev = 1;
        return;
    }

    // Signed, wraparound-safe deltas since the previous tick
    int32_t stepper_delta = (int32_t)(stepper_pos - cls->prev_stepper_pos);
    int32_t angle_delta = (int16_t)(angle_raw - cls->prev_angle_raw);
    cls->prev_angle_raw = angle_raw;
    cls->prev_stepper_pos = stepper_pos;

    // ratio_q16 is already Q16.16, so raw_delta * ratio_q16 directly yields
    // a Q16.16 steps value (int64 intermediate avoids overflow).
    int32_t angle_delta_steps_q16 =
        (int32_t)((int64_t)angle_delta * cls->ratio_q16);
    int32_t stepper_delta_q16 = (int32_t)((uint32_t)stepper_delta << 16);
    cls->error_accum_q16 += stepper_delta_q16 - angle_delta_steps_q16;

    uint32_t now = timer_read_time();
    uint32_t abs_error = cls->error_accum_q16 < 0
        ? -cls->error_accum_q16 : cls->error_accum_q16;
    if (abs_error >= cls->deadband_q16
        && now - cls->last_correction_time >= cls->min_interval_ticks) {
        int_fast8_t want_increase = cls->error_accum_q16 > 0;
        stepper_apply_correction_step(cls->stepper, want_increase);
        cls->error_accum_q16 += want_increase ? -(1<<16) : (1<<16);
        cls->corrected_steps += want_increase ? 1 : -1;
        cls->last_correction_time = now;
    }
}

// Background task that runs the correction loop for all configured oids
void
closed_loop_stepper_task(void)
{
    if (!sched_check_wake(&closed_loop_wake))
        return;
    uint8_t oid;
    struct closed_loop_stepper *cls;
    foreach_oid(oid, cls, command_config_closed_loop_stepper) {
        if (!(cls->flags & CLS_PENDING))
            continue;
        cls->flags &= ~CLS_PENDING;
        closed_loop_stepper_check(cls);
    }
}
DECL_TASK(closed_loop_stepper_task);
