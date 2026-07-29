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
// The angle sample is optionally passed through the same 64-bucket linear
// calibration table klippy/extras/angle.py's AngleCalibration builds (via
// ANGLE_CALIBRATE), uploaded once at config time - see apply_calibration()
// below, which is an integer-only port of AngleCalibration.apply_calibration().
// The signed "ratio" gain uploaded at config time is expected to absorb the
// axis's direction sense (independent of calibration_reversed, which instead
// mirrors AngleCalibration's own sensor-orientation flip); this is a coarse
// slip corrector, not a precision position source.

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_angle.h" // spi_angle_get_latest
#include "stepper.h" // stepper_get_position

// Matches klippy/extras/angle.py's CALIBRATION_BITS/ANGLE_BITS constants
#define CAL_BUCKETS 64
#define CAL_TABLE_SIZE (CAL_BUCKETS + 1)
#define CAL_INTERP_BITS 10             // ANGLE_BITS(16) - CALIBRATION_BITS(6)
#define CAL_INTERP_MASK ((1u << CAL_INTERP_BITS) - 1)
#define CAL_INTERP_ROUND (1 << (CAL_INTERP_BITS - 1))

struct closed_loop_stepper {
    struct timer timer;
    uint32_t rest_ticks;
    struct stepper *stepper;
    struct spi_angle *angle;

    // Config (uploaded once at config_closed_loop_stepper time)
    int32_t ratio_q16;          // signed Q16.16 steps-per-angle-count, where
                                 // "angle-count" assumes a full 65536-count
                                 // revolution (matches angle.py's ANGLE_BITS)
    uint32_t deadband_q16;      // Q16.16 steps - accumulated error threshold
    uint32_t min_interval_ticks; // rate limiter: min ticks between corrections
    uint8_t calibration_reversed;

    // Calibration table (uploaded via config_closed_loop_stepper_calibration,
    // one entry at a time; has_calibration stays 0 - bypassing the lookup -
    // until at least one entry has actually been uploaded)
    int32_t calibration[CAL_TABLE_SIZE];
    uint8_t has_calibration;

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
    cls->calibration_reversed = args[6];
}
DECL_COMMAND(command_config_closed_loop_stepper,
             "config_closed_loop_stepper oid=%c stepper_oid=%c angle_oid=%c"
             " ratio=%i deadband=%u min_interval_ticks=%u"
             " calibration_reversed=%c");

// Upload one calibration table entry (sent CAL_TABLE_SIZE times at config
// time by closed_loop_stepper.py, mirroring AngleCalibration.calibration).
void
command_config_closed_loop_stepper_calibration(uint32_t *args)
{
    struct closed_loop_stepper *cls = oid_lookup(
        args[0], command_config_closed_loop_stepper);
    uint8_t index = args[1];
    if (index >= CAL_TABLE_SIZE)
        shutdown("Invalid closed_loop_stepper calibration index");
    cls->calibration[index] = args[2];
    cls->has_calibration = 1;
}
DECL_COMMAND(command_config_closed_loop_stepper_calibration,
             "config_closed_loop_stepper_calibration oid=%c index=%c"
             " value=%i");

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

// Integer port of klippy/extras/angle.py's AngleCalibration.apply_calibration
// for a single 16-bit raw sample (no multi-turn accumulation needed here -
// the caller only ever uses wraparound-safe deltas between two calibrated
// samples, so working directly in the 0..65535 domain is sufficient).
static uint32_t
apply_calibration(struct closed_loop_stepper *cls, uint32_t angle)
{
    if (!cls->has_calibration)
        return angle;
    uint32_t bucket = (angle & 0xffff) >> CAL_INTERP_BITS;
    int32_t cal1 = cls->calibration[bucket];
    int32_t cal2 = cls->calibration[bucket + 1];
    int32_t frac = (int32_t)(angle & CAL_INTERP_MASK);
    int32_t adj = cal1 + ((frac * (cal2 - cal1) + CAL_INTERP_ROUND)
                          >> CAL_INTERP_BITS);
    uint32_t angle_diff = ((uint32_t)adj - angle) & 0xffff;
    int32_t signed_diff = (int16_t)angle_diff;
    uint32_t new_angle = (angle + (uint32_t)signed_diff) & 0xffff;
    if (cls->calibration_reversed)
        new_angle = (uint32_t)(-(int32_t)new_angle) & 0xffff;
    return new_angle;
}

// Run one correction cycle for a single stepper/angle pair
static void
closed_loop_stepper_check(struct closed_loop_stepper *cls)
{
    uint32_t angle_time, angle_raw;
    if (!spi_angle_get_latest(cls->angle, &angle_time, &angle_raw))
        return;
    angle_raw = apply_calibration(cls, angle_raw);

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
