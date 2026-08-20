# Realtime closed-loop stepper correction using SPI magnetic angle sensors
#
# Copyright (C) 2026  Adam Barnas
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Host-side configuration/telemetry wrapper around the firmware corrector in
# src/closed_loop_stepper.c.  All the actual realtime work (comparing the
# stepper's commanded position against the angle sensor and injecting
# correction steps) happens on the MCU; this module only uploads gains once
# at connect time, starts/stops the MCU task, and polls status for logging /
# Moonraker.  Complements (does not replace) the existing idle-only
# klippy/extras/closed_loop.py.
#
# Config:
#   [closed_loop_stepper x]
#   stepper: stepper_x               # defaults to stepper_<axis>
#   sensor: x_angle_sensor           # name of the [angle] section (no "angle " prefix)
#   ratio: <float>                   # optional: signed steps-per-raw-angle-count
#                                     #   override; auto-derived from the
#                                     #   stepper's rotation_distance otherwise
#   invert_ratio: False              # flip the sign of the auto-derived ratio
#   deadband_steps: 2.0              # error (in steps) below which nothing acts
#                                     #   (noise/hysteresis floor, all modes)
#   max_correction_steps_per_sec: 200 # hard rate limiter on injected steps
#   poll_interval: <float>           # seconds between MCU correction checks;
#                                     #   defaults to the angle sensor's own
#                                     #   sample_period
#   enable_on_start: True
#
#   control_mode: bang_bang          # bang_bang (default, relay/hysteresis,
#                                     #   unchanged legacy behaviour) | p | pid
#   kp: 0.0                          # proportional gain (steps out per step
#                                     #   of error); required (>0) for p/pid.
#                                     #   Capped at 2.0: the discrete recursion
#                                     #   e[k+1]=(1-kp)*e[k] this reduces to in
#                                     #   isolation is only stable for kp<2.
#   ki: 0.0                          # integral gain; pid mode only, capped at
#                                     #   1.0. Clamped anti-windup accumulator,
#                                     #   see max_integral_steps.
#   kd: 0.0                          # derivative gain; pid mode only, capped
#                                     #   at 2.0. Runs through a fixed internal
#                                     #   low-pass filter (raw angle is noisy).
#   max_integral_steps: <float>      # anti-windup clamp on the I accumulator;
#                                     #   defaults to 10x deadband_steps
#   max_error_steps: <float>         # panic/watchdog threshold on the
#                                     #   accumulated error - correction is
#                                     #   latched off (fault) if ever exceeded,
#                                     #   in any mode; defaults to
#                                     #   max(50, 25x deadband_steps)
#
# GCode commands (AXIS=<name> selects the instance):
#   CLOSED_LOOP_STEPPER_STATUS    AXIS=x
#   CLOSED_LOOP_STEPPER_ENABLE    AXIS=x ENABLE=0|1  # ENABLE=1 also clears a
#                                                     # latched fault (full
#                                                     # state reset on the MCU)
#   CLOSED_LOOP_STEPPER_SET_MODE  AXIS=x [MODE=bang_bang|p|pid] [KP=<float>]
#                                  [KI=<float>] [KD=<float>]
#     Changes control_mode/kp/ki/kd on the fly, no RESTART needed - any
#     parameter left out keeps its current value. Does NOT clear a latched
#     fault (use CLOSED_LOOP_STEPPER_ENABLE for that) and does NOT touch
#     deadband_steps/max_integral_steps/max_error_steps/
#     max_correction_steps_per_sec, which remain config-only.

import logging

MIN_MSG_TIME = 0.100
STATUS_POLL_TIME = 1.0
ANGLE_BITS = 16  # angle module normalises all sensors to 0..65535 per revolution

CONTROL_MODES = {'bang_bang': 0, 'p': 1, 'pid': 2}


def _check_gains(mode_name, kp, ki, kd):
    # Shared between config-time validation (__init__) and runtime
    # validation (CLOSED_LOOP_STEPPER_SET_MODE) so the two can never drift
    # apart. Returns an error message string if the combination is unsafe/
    # nonsensical, or None if it's fine. (kp/ki/kd upper bounds themselves
    # are enforced by the minval/maxval on the respective getfloat/get_float
    # calls at each call site, not here.)
    if mode_name == 'bang_bang':
        return None
    if kp <= 0.:
        return "control_mode=%s requires kp > 0" % (mode_name,)
    if mode_name == 'p' and (ki or kd):
        return ("control_mode=p does not use ki/kd - set control_mode=pid"
                " to use them")
    return None


class ClosedLoopStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]   # 'x' / 'y' from [closed_loop_stepper x]

        axis = config.get('axis', self.name).lower()
        self._stepper_name = config.get('stepper', 'stepper_%s' % axis)
        self._sensor_name = config.get('sensor', '%s_angle_sensor' % axis)

        self._ratio_override = config.getfloat('ratio', None)
        self._invert_ratio = config.getboolean('invert_ratio', False)
        self._deadband_steps = config.getfloat('deadband_steps', 2.0, above=0.)
        self._max_correction_rate = config.getfloat(
            'max_correction_steps_per_sec', 200., above=0.)
        self._poll_interval = config.getfloat('poll_interval', None, above=0.)
        self._enable_on_start = config.getboolean('enable_on_start', True)

        self._control_mode_name = config.getchoice(
            'control_mode', CONTROL_MODES, default='bang_bang')
        # kp/ki/kd upper bounds aren't arbitrary: kp<=2.0 is the stability
        # bound of the discrete recursion e[k+1]=(1-kp)*e[k] that a P-only
        # controller reduces to here (deadbeat at kp=1, oscillating but still
        # bounded up to kp=2). ki/kd have no equally clean closed-form bound
        # given the combined loop, so their caps are deliberately conservative
        # given this MCU has no FPU and only sees the raw, uncalibrated angle.
        self._kp = config.getfloat('kp', 0., minval=0., maxval=2.0)
        self._ki = config.getfloat('ki', 0., minval=0., maxval=1.0)
        self._kd = config.getfloat('kd', 0., minval=0., maxval=2.0)
        self._max_integral_steps = config.getfloat(
            'max_integral_steps', None, above=0.)
        self._max_error_steps = config.getfloat(
            'max_error_steps', None, above=0.)

        if self._control_mode_name == 'bang_bang':
            if self._kp or self._ki or self._kd:
                logging.warning(
                    "closed_loop_stepper %s: kp/ki/kd are set but"
                    " control_mode=bang_bang ignores them (set"
                    " control_mode=p or control_mode=pid to use them)",
                    self.name)
        else:
            err = _check_gains(
                self._control_mode_name, self._kp, self._ki, self._kd)
            if err:
                raise self.printer.config_error(
                    "closed_loop_stepper %s: %s" % (self.name, err))
        if self._max_integral_steps is None:
            self._max_integral_steps = 10. * self._deadband_steps
        if self._max_error_steps is None:
            self._max_error_steps = max(50., 25. * self._deadband_steps)

        self._mcu = None
        self._mcu_stepper = None
        self._angle_obj = None
        self._angle_oid = None
        self._angle_subscribed = False
        self._oid = None
        self._cmd_queue = None
        self._query_cmd = None
        self._get_state_cmd = None
        self._set_gains_cmd = None
        self._reactor = self.printer.get_reactor()
        self._status_timer = None

        self._enabled = False
        self._corrected_steps = 0
        self._error_steps = 0.
        self._fault = False

        # klippy:mcu_identify fires after all config sections (including
        # toolhead/kinematics/steppers) are constructed, but strictly before
        # the klippy:connect dispatch where MCU.finalize_config() runs - see
        # extras/tmc.py's _handle_mcu_identify for the same pattern. Doing our
        # create_oid()/register_config_callback() any later (e.g. in a
        # klippy:connect handler) would race the target MCU's own
        # klippy:connect handler, which may already have finalized its config
        # by the time ours runs, silently dropping our config commands.
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'CLOSED_LOOP_STEPPER_STATUS', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_STEPPER_STATUS,
            desc="Report realtime closed-loop stepper correction status")
        gcode.register_mux_command(
            'CLOSED_LOOP_STEPPER_ENABLE', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_STEPPER_ENABLE,
            desc="Enable or disable realtime closed-loop stepper correction"
                 " (ENABLE=0|1)")
        gcode.register_mux_command(
            'CLOSED_LOOP_STEPPER_SET_MODE', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_STEPPER_SET_MODE,
            desc="Change control_mode/kp/ki/kd at runtime, no RESTART"
                 " needed (MODE=bang_bang|p|pid KP= KI= KD=)")

    # =========================================================================
    # Setup
    # =========================================================================

    def _handle_mcu_identify(self):
        # force_move's stepper registry is populated as each MCU_stepper is
        # constructed (during toolhead/kinematics setup) and is the same
        # lookup extras/tmc.py uses here for exactly this reason.
        force_move = self.printer.lookup_object('force_move')
        self._mcu_stepper = force_move.lookup_stepper(self._stepper_name)
        self._mcu = self._mcu_stepper.get_mcu()

        angle_obj = self.printer.lookup_object('angle %s' % self._sensor_name)
        if angle_obj.mcu is not self._mcu:
            raise self.printer.config_error(
                "closed_loop_stepper %s: sensor '%s' must be on the same mcu"
                " as stepper '%s'" % (self.name, self._sensor_name,
                                      self._stepper_name))
        self._angle_obj = angle_obj
        self._angle_oid = angle_obj.oid
        self._angle_subscribed = False
        if self._poll_interval is None:
            self._poll_interval = angle_obj.sample_period

        ratio = self._ratio_override
        if ratio is None:
            rotation_dist, steps_per_rotation = \
                self._mcu_stepper.get_rotation_distance()
            # Matches the same steps-per-angle-count convention already used
            # by angle.py's calibration code (angle_to_mcu_pos).
            ratio = steps_per_rotation / float(1 << ANGLE_BITS)
            if self._invert_ratio:
                ratio = -ratio
        self._ratio_q16 = int(round(ratio * 65536.))
        self._deadband_q16 = int(round(self._deadband_steps * 65536.))
        self._control_mode = CONTROL_MODES[self._control_mode_name]
        self._kp_q16 = int(round(self._kp * 65536.))
        self._ki_q16 = int(round(self._ki * 65536.))
        self._kd_q16 = int(round(self._kd * 65536.))
        self._max_integral_q16 = int(round(self._max_integral_steps * 65536.))
        self._panic_error_q16 = int(round(self._max_error_steps * 65536.))

        self._oid = self._mcu.create_oid()
        self._cmd_queue = self._mcu.alloc_command_queue()
        self._mcu.register_config_callback(self._build_config)

        self._status_timer = self._reactor.register_timer(
            self._status_poll_timer,
            self._reactor.monotonic() + STATUS_POLL_TIME)

    def _build_config(self):
        stepper_oid = self._mcu_stepper.get_oid()
        min_interval_ticks = max(1, int(self._mcu.seconds_to_clock(
            1.0 / self._max_correction_rate)))
        # P/PID mode isn't gated per-step like bang-bang - it can apply
        # several correction steps in a single check cycle. Cap that count so
        # max_correction_steps_per_sec still means the same thing (a hard
        # ceiling on injected steps/sec) regardless of control_mode.
        max_steps_per_check = max(1, int(round(
            self._max_correction_rate * self._poll_interval)))
        self._mcu.add_config_cmd(
            "config_closed_loop_stepper oid=%d stepper_oid=%d angle_oid=%d"
            " ratio=%d deadband=%d min_interval_ticks=%d mode=%d kp=%d ki=%d"
            " kd=%d max_integral=%d panic_error=%d max_steps_per_check=%d"
            % (self._oid, stepper_oid, self._angle_oid,
               self._ratio_q16, self._deadband_q16, min_interval_ticks,
               self._control_mode, self._kp_q16, self._ki_q16, self._kd_q16,
               self._max_integral_q16, self._panic_error_q16,
               max_steps_per_check))
        self._mcu.add_config_cmd(
            "query_closed_loop_stepper oid=%d clock=0 rest_ticks=0"
            % (self._oid,), on_restart=True)
        self._query_cmd = self._mcu.lookup_command(
            "query_closed_loop_stepper oid=%c clock=%u rest_ticks=%u",
            cq=self._cmd_queue)
        self._get_state_cmd = self._mcu.lookup_query_command(
            "closed_loop_stepper_get_state oid=%c",
            "closed_loop_stepper_state oid=%c corrected_steps=%i error_q16=%i"
            " fault=%c",
            oid=self._oid, cq=self._cmd_queue)
        self._set_gains_cmd = self._mcu.lookup_command(
            "set_closed_loop_stepper_gains oid=%c mode=%c kp=%i ki=%i kd=%i",
            cq=self._cmd_queue)
        logging.info(
            "closed_loop_stepper %s: connected - stepper=%s sensor='angle %s'"
            " control_mode=%s ratio_q16=%d deadband_q16=%d"
            " min_interval_ticks=%d kp_q16=%d ki_q16=%d kd_q16=%d"
            " max_integral_q16=%d panic_error_q16=%d max_steps_per_check=%d",
            self.name, self._stepper_name, self._sensor_name,
            self._control_mode_name, self._ratio_q16, self._deadband_q16,
            min_interval_ticks, self._kp_q16, self._ki_q16, self._kd_q16,
            self._max_integral_q16, self._panic_error_q16,
            max_steps_per_check)

    def _handle_ready(self):
        if self._enable_on_start:
            self._send_enable(True)

    # =========================================================================
    # MCU start/stop
    # =========================================================================

    def _angle_batch_handler(self, msg):
        # We don't use the streamed samples themselves - the firmware task
        # reads its own "latest sample" side-channel directly off the MCU.
        # Subscribing here is what keeps the angle sensor's SPI polling loop
        # running at all: bulk_sensor.BatchBulkHelper only samples while at
        # least one client is registered (angle.py's add_client()), which is
        # also why closed_loop.py subscribes for its own idle-jog checks.
        # Returning False here drops our subscription once disabled.
        if not self._enabled:
            self._angle_subscribed = False
            return False
        return True

    def _send_enable(self, enable):
        if enable:
            if not self._angle_subscribed:
                self._angle_subscribed = True
                self._angle_obj.add_client(self._angle_batch_handler)
            systime = self._reactor.monotonic()
            print_time = self._mcu.estimated_print_time(systime) + MIN_MSG_TIME
            reqclock = self._mcu.print_time_to_clock(print_time)
            rest_ticks = self._mcu.seconds_to_clock(self._poll_interval)
            self._query_cmd.send([self._oid, reqclock, rest_ticks],
                                 reqclock=reqclock)
        else:
            self._query_cmd.send_wait_ack([self._oid, 0, 0])
        self._enabled = enable

    # =========================================================================
    # Status polling (for logging / get_status / gcode reporting)
    # =========================================================================

    def _update_state(self, params):
        self._corrected_steps = params['corrected_steps']
        self._error_steps = params['error_q16'] / 65536.
        fault = bool(params.get('fault', 0))
        if fault and not self._fault:
            logging.warning(
                "closed_loop_stepper %s: FAULT latched on MCU (error"
                " exceeded max_error_steps=%.3f) - correction stopped;"
                " run CLOSED_LOOP_STEPPER_ENABLE AXIS=%s ENABLE=1 to"
                " investigate and reset",
                self.name, self._max_error_steps, self.name)
        self._fault = fault

    def _status_poll_timer(self, eventtime):
        if self._enabled and self._get_state_cmd is not None:
            self._update_state(self._get_state_cmd.send([self._oid]))
        return eventtime + STATUS_POLL_TIME

    # =========================================================================
    # GCode commands
    # =========================================================================

    def cmd_CLOSED_LOOP_STEPPER_STATUS(self, gcmd):
        if self._get_state_cmd is not None and self._enabled:
            self._update_state(self._get_state_cmd.send([self._oid]))
        gcmd.respond_info(
            "CLOSED_LOOP_STEPPER [%s]  mode=%-9s  enabled=%-5s"
            "  fault=%-5s  corrected_steps=%-8s  error=%-10s"
            % (self.name, self._control_mode_name, self._enabled,
               self._fault, self._corrected_steps,
               "%.3f steps" % self._error_steps))

    def cmd_CLOSED_LOOP_STEPPER_ENABLE(self, gcmd):
        enable = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        self._send_enable(enable)
        gcmd.respond_info(
            "CLOSED_LOOP_STEPPER [%s]: %s"
            % (self.name, "enabled" if enable else "disabled"))

    def cmd_CLOSED_LOOP_STEPPER_SET_MODE(self, gcmd):
        if self._set_gains_cmd is None:
            raise gcmd.error(
                "closed_loop_stepper %s: not yet connected to its MCU"
                % (self.name,))
        mode_name = gcmd.get('MODE', self._control_mode_name)
        if mode_name not in CONTROL_MODES:
            raise gcmd.error(
                "closed_loop_stepper %s: invalid MODE '%s' (must be one of:"
                " %s)" % (self.name, mode_name,
                          ', '.join(sorted(CONTROL_MODES))))
        # Same bounds as the config-time kp/ki/kd (see __init__) - kp<=2.0 is
        # the stability bound of the P-only recursion this control law
        # reduces to; ki/kd caps are conservative for the same reasons noted
        # there.
        kp = gcmd.get_float('KP', self._kp, minval=0., maxval=2.0)
        ki = gcmd.get_float('KI', self._ki, minval=0., maxval=1.0)
        kd = gcmd.get_float('KD', self._kd, minval=0., maxval=2.0)
        err = _check_gains(mode_name, kp, ki, kd)
        if err:
            raise gcmd.error("closed_loop_stepper %s: %s" % (self.name, err))
        if mode_name == 'bang_bang' and (kp or ki or kd):
            gcmd.respond_info(
                "closed_loop_stepper %s: kp/ki/kd set but"
                " control_mode=bang_bang ignores them" % (self.name,))

        self._control_mode_name = mode_name
        self._control_mode = CONTROL_MODES[mode_name]
        self._kp, self._ki, self._kd = kp, ki, kd
        self._kp_q16 = int(round(kp * 65536.))
        self._ki_q16 = int(round(ki * 65536.))
        self._kd_q16 = int(round(kd * 65536.))
        self._set_gains_cmd.send(
            [self._oid, self._control_mode, self._kp_q16, self._ki_q16,
             self._kd_q16])
        gcmd.respond_info(
            "CLOSED_LOOP_STEPPER [%s]: control_mode=%s kp=%.4f ki=%.4f"
            " kd=%.4f" % (self.name, mode_name, kp, ki, kd))

    # =========================================================================
    # Moonraker / status API
    # =========================================================================

    def get_status(self, eventtime=None):
        return {
            'enabled': self._enabled,
            'control_mode': self._control_mode_name,
            'fault': self._fault,
            'corrected_steps': self._corrected_steps,
            'error_steps': self._error_steps,
        }


def load_config_prefix(config):
    return ClosedLoopStepper(config)
