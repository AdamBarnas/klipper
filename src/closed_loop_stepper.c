// Realtime closed-loop stepper correction using SPI magnetic angle sensors
//
// Copyright (C) 2026  Adam Barnas
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Every tick this compares how far the stepper's own commanded position
// moved against how far the angle sensor says the axis actually moved, and
// accumulates the (signed) difference in a Q16.16 fixed-point register
// (error_accum_q16 - this is a running absolute position error, not just a
// per-tick delta). What happens with that error then depends on
// control_mode:
//
//  - CLS_MODE_BANG_BANG (default, unchanged from the original corrector):
//    whenever the accumulated error exceeds a configured deadband and a
//    rate limiter allows it, inject a single out-of-band correction step
//    via stepper_apply_correction_step() (see stepper.c) in whichever
//    direction closes the gap. No gains involved - see closed_loop.tex
//    sec.5 for why this is a relay/hysteresis controller, not a P/PID one.
//
//  - CLS_MODE_P / CLS_MODE_PID: a real discrete control law (P, or P+I+D)
//    computed on error_accum_q16, saturated to an integer number of
//    correction steps per check cycle. See
//    closed_loop_stepper_apply_pid() below for the safety measures this
//    needs (anti-windup clamp, D-term filtering, output cap) that a plain
//    relay controller doesn't.
//
// In every mode, a panic/watchdog check on the accumulated error runs
// first and latches a fault (stopping further correction) if it is ever
// exceeded - see the CLS_FAULT handling in closed_loop_stepper_check().
//
// control_mode/kp/ki/kd can be changed at runtime (no MCU reconnect needed)
// via set_closed_loop_stepper_gains - everything else (deadband, the
// panic threshold, rate limits) is config-only.
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
    int32_t ratio_q16;          // signed Q16.16 steps-per-angle-count, where
                                 // "angle-count" assumes a full 65536-count
                                 // revolution (matches angle.py's ANGLE_BITS)
    uint32_t deadband_q16;      // Q16.16 steps - error threshold below which
                                 // no mode acts (hysteresis/noise gate)
    uint32_t min_interval_ticks; // bang-bang only: min ticks between steps
    uint8_t angle_shift;        // normalises a sensor's native angle range up
                                 // to the full 65536-count assumption above
                                 // (0 for 16-bit chips, 2 for mt6835's 14-bit
                                 // native range) - see spi_angle_get_angle_bits()
    uint8_t control_mode;       // CLS_MODE_BANG_BANG / CLS_MODE_P / CLS_MODE_PID
    int32_t kp_q16, ki_q16, kd_q16; // Q16.16 gains (P/PID only)
    uint32_t max_integral_q16;  // P/PID only: anti-windup clamp on the
                                 // integral accumulator (Q16.16 steps)
    uint32_t panic_error_q16;   // all modes: |error| watchdog threshold -
                                 // latches CLS_FAULT and stops correcting
    uint32_t max_steps_per_check; // P/PID only: hard cap on |steps| applied
                                 // in a single check cycle

    // Live tracking state (reset whenever (re)started via the query command)
    uint32_t prev_angle_raw;
    uint32_t prev_stepper_pos;
    int32_t error_accum_q16;
    int32_t error_prev_q16;     // P/PID only: previous error, for D term
    int32_t integral_accum_q16; // P/PID only: clamped running integral
    int32_t d_filt_q16;         // P/PID only: low-pass filtered D term
    uint32_t last_correction_time;
    int32_t corrected_steps;    // cumulative signed correction step count
    uint8_t flags, have_prev;
};

enum {
    CLS_PENDING = 1<<0,
    CLS_FAULT   = 1<<1,
};

enum {
    CLS_MODE_BANG_BANG = 0,
    CLS_MODE_P         = 1,
    CLS_MODE_PID       = 2,
};

// Single-pole low-pass filter coefficient for the D term (fixed, not
// user-configurable, to keep the config surface small): filt +=
// (raw-filt)*ALPHA. 0.25 in Q16.16.
#define CLS_D_FILTER_ALPHA_Q16 16384

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
    cls->angle_shift = 16 - spi_angle_get_angle_bits(cls->angle);
    cls->control_mode = args[6];
    cls->kp_q16 = args[7];
    cls->ki_q16 = args[8];
    cls->kd_q16 = args[9];
    cls->max_integral_q16 = args[10];
    cls->panic_error_q16 = args[11];
    cls->max_steps_per_check = args[12];
}
DECL_COMMAND(command_config_closed_loop_stepper,
             "config_closed_loop_stepper oid=%c stepper_oid=%c angle_oid=%c"
             " ratio=%i deadband=%u min_interval_ticks=%u mode=%c kp=%i"
             " ki=%i kd=%i max_integral=%u panic_error=%u"
             " max_steps_per_check=%u");

// Start/stop periodic correction.  rest_ticks==0 stops and clears state.
// Also used to clear a latched CLS_FAULT: any call (start or stop) resets
// all live state, including the fault flag via flags=0 below - so a host
// re-enable after a fault is a normal restart, not a special code path.
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
    cls->error_prev_q16 = 0;
    cls->integral_accum_q16 = 0;
    cls->d_filt_q16 = 0;
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

// Change control_mode/kp/ki/kd on an already-configured instance without a
// full config_closed_loop_stepper re-send (that command is config-only -
// see closed_loop_stepper.py's use of add_config_cmd - so it can only run
// once per MCU connection). This one uses oid_lookup, not oid_alloc, so
// it's a normal runtime command like query_closed_loop_stepper and can be
// sent any time. Does NOT touch deadband/max_integral/panic_error/
// max_steps_per_check or clear a latched CLS_FAULT - those still need a
// query_closed_loop_stepper reset (CLOSED_LOOP_STEPPER_ENABLE) to change/
// clear, so a mode/gain change can never silently resume a faulted axis.
void
command_set_closed_loop_stepper_gains(uint32_t *args)
{
    uint8_t oid = args[0];
    struct closed_loop_stepper *cls = oid_lookup(
        oid, command_config_closed_loop_stepper);
    cls->control_mode = args[1];
    cls->kp_q16 = args[2];
    cls->ki_q16 = args[3];
    cls->kd_q16 = args[4];
    // Old integral/D-filter state was accumulated under the previous gains
    // (or a different mode entirely) - carrying it over would produce a
    // discontinuous jump in the very next correction. error_prev_q16 is set
    // to the *current* error (not zeroed) so the first D computation under
    // the new gains sees a zero delta instead of a spurious spike.
    cls->integral_accum_q16 = 0;
    cls->d_filt_q16 = 0;
    cls->error_prev_q16 = cls->error_accum_q16;
}
DECL_COMMAND(command_set_closed_loop_stepper_gains,
             "set_closed_loop_stepper_gains oid=%c mode=%c kp=%i ki=%i"
             " kd=%i");

void
command_closed_loop_stepper_get_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct closed_loop_stepper *cls = oid_lookup(
        oid, command_config_closed_loop_stepper);
    sendf("closed_loop_stepper_state oid=%c corrected_steps=%i error_q16=%i"
          " fault=%c",
          oid, cls->corrected_steps, cls->error_accum_q16,
          !!(cls->flags & CLS_FAULT));
}
DECL_COMMAND(command_closed_loop_stepper_get_state,
             "closed_loop_stepper_get_state oid=%c");

// Original relay/hysteresis corrector: always exactly one step, gated by
// deadband and a fixed minimum interval. No gains, no proportionality to
// the error magnitude - see closed_loop.tex sec.5.
static void
closed_loop_stepper_apply_bang_bang(struct closed_loop_stepper *cls)
{
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

// Discrete P (or P+I+D, for CLS_MODE_PID) control law on error_accum_q16
// (the running absolute position error, Q16.16 steps). Runs once per check
// cycle (every rest_ticks / poll_interval) and outputs an integer number of
// correction steps to apply THIS cycle.
//
// Safety measures, each corresponding to a point in closed_loop.tex sec.6:
//  - deadband_q16 below still gates action (noise/hysteresis floor), same
//    as bang-bang.
//  - the integral accumulator is clamped to +-max_integral_q16
//    (anti-windup): without this, time spent saturated at
//    max_steps_per_check would let the I term grow without bound.
//  - the D term runs through a fixed single-pole low-pass filter before
//    being multiplied by kd_q16, because it acts on the raw/uncalibrated
//    angle sample and is the term most sensitive to sensor noise.
//  - the combined P+I+D output is rounded to an integer step count and
//    hard-capped at +-max_steps_per_check before being applied - gains
//    can be tuned badly, but they can never inject more than this many
//    steps in one cycle.
// The all-modes panic/watchdog check in closed_loop_stepper_check() below
// is the last line of defense on top of these.
static void
closed_loop_stepper_apply_pid(struct closed_loop_stepper *cls)
{
    int32_t e = cls->error_accum_q16;
    uint32_t abs_e = e < 0 ? -e : e;
    if (abs_e < cls->deadband_q16) {
        // Below the noise gate: don't act, but keep error_prev_q16 current
        // so the D term doesn't see a spurious jump next time we do act.
        cls->error_prev_q16 = e;
        return;
    }

    // P term: e and kp_q16 are both Q16.16, so their product is Q32.32 -
    // shift back down to Q16.16 (same pattern as the ratio_q16 multiply
    // above, just with both operands fixed-point instead of one).
    int32_t p_term = (int32_t)(((int64_t)e * cls->kp_q16) >> 16);

    int32_t i_term = 0;
    if (cls->control_mode == CLS_MODE_PID && cls->ki_q16) {
        int32_t max_i = (int32_t)cls->max_integral_q16;
        int64_t i_accum = (int64_t)cls->integral_accum_q16 + e;
        if (i_accum > max_i)
            i_accum = max_i;
        else if (i_accum < -max_i)
            i_accum = -max_i;
        cls->integral_accum_q16 = (int32_t)i_accum;
        i_term = (int32_t)(((int64_t)cls->integral_accum_q16 * cls->ki_q16)
                            >> 16);
    }

    int32_t d_term = 0;
    if (cls->control_mode == CLS_MODE_PID && cls->kd_q16) {
        int32_t raw_delta = e - cls->error_prev_q16;
        cls->d_filt_q16 += (int32_t)(
            ((int64_t)(raw_delta - cls->d_filt_q16)
             * CLS_D_FILTER_ALPHA_Q16) >> 16);
        d_term = (int32_t)(((int64_t)cls->d_filt_q16 * cls->kd_q16) >> 16);
    }
    cls->error_prev_q16 = e;

    int32_t u_q16 = p_term + i_term + d_term;
    // Round to nearest integer step count (symmetric around zero).
    int32_t steps = u_q16 >= 0 ? (u_q16 + (1<<15)) >> 16
                               : -(((-u_q16) + (1<<15)) >> 16);

    int32_t cap = (int32_t)cls->max_steps_per_check;
    if (steps > cap)
        steps = cap;
    else if (steps < -cap)
        steps = -cap;
    if (!steps)
        return;

    int_fast8_t want_increase = steps > 0;
    uint32_t count = want_increase ? (uint32_t)steps : (uint32_t)(-steps);
    uint32_t i;
    for (i = 0; i < count; i++)
        stepper_apply_correction_step(cls->stepper, want_increase);

    int32_t applied_q16 = (int32_t)(count << 16);
    cls->error_accum_q16 -= want_increase ? applied_q16 : -applied_q16;
    cls->corrected_steps += want_increase ? (int32_t)count : -(int32_t)count;
}

// Run one correction cycle for a single stepper/angle pair
static void
closed_loop_stepper_check(struct closed_loop_stepper *cls)
{
    if (cls->flags & CLS_FAULT)
        // Latched fault: do nothing until a host CLOSED_LOOP_STEPPER_ENABLE
        // re-issues query_closed_loop_stepper and resets state.
        return;

    uint32_t angle_time, angle_raw;
    if (!spi_angle_get_latest(cls->angle, &angle_time, &angle_raw))
        return;
    // Normalise to a full 0..65535 range so both the wraparound diff below
    // and ratio_q16 (which assumes a 65536-count revolution) are correct
    // regardless of the sensor's native bit width (e.g. mt6835 only fills
    // the bottom 14 bits - without this, a rollover there reads as a huge
    // spurious ~2^14-count jump instead of a small in-range step, and even
    // non-rollover moves would be undercounted by 4x).
    angle_raw <<= cls->angle_shift;

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

    // Watchdog: applies to every mode. A mis-signed ratio (or bad P/PID
    // gains) can make corrections make the error worse instead of better;
    // rather than keep injecting steps in a possibly-wrong direction until
    // someone notices, latch a fault and stop. This is the direct fix for
    // the "sign-flip runaway" incident referenced in closed_loop.tex sec.5.
    uint32_t abs_error = cls->error_accum_q16 < 0
        ? -cls->error_accum_q16 : cls->error_accum_q16;
    if (abs_error >= cls->panic_error_q16) {
        cls->flags |= CLS_FAULT;
        return;
    }

    if (cls->control_mode == CLS_MODE_BANG_BANG)
        closed_loop_stepper_apply_bang_bang(cls);
    else
        closed_loop_stepper_apply_pid(cls);
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
