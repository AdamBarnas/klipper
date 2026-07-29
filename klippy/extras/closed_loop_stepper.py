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
#   deadband_steps: 2.0              # accumulated error (in steps) before correcting
#   max_correction_steps_per_sec: 200 # hard rate limiter on injected steps
#   poll_interval: <float>           # seconds between MCU correction checks;
#                                     #   defaults to the angle sensor's own
#                                     #   sample_period
#   enable_on_start: True
#
# GCode commands (AXIS=<name> selects the instance):
#   CLOSED_LOOP_STEPPER_STATUS  AXIS=x
#   CLOSED_LOOP_STEPPER_ENABLE  AXIS=x ENABLE=0|1

import logging

MIN_MSG_TIME = 0.100
STATUS_POLL_TIME = 1.0
ANGLE_BITS = 16  # angle module normalises all sensors to 0..65535 per revolution


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

        self._mcu = None
        self._mcu_stepper = None
        self._angle_obj = None
        self._angle_oid = None
        self._angle_subscribed = False
        self._oid = None
        self._cmd_queue = None
        self._query_cmd = None
        self._get_state_cmd = None
        self._reactor = self.printer.get_reactor()
        self._status_timer = None

        self._enabled = False
        self._corrected_steps = 0
        self._error_steps = 0.

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
        self._mcu.add_config_cmd(
            "config_closed_loop_stepper oid=%d stepper_oid=%d angle_oid=%d"
            " ratio=%d deadband=%d min_interval_ticks=%d"
            % (self._oid, stepper_oid, self._angle_oid,
               self._ratio_q16, self._deadband_q16, min_interval_ticks))
        self._mcu.add_config_cmd(
            "query_closed_loop_stepper oid=%d clock=0 rest_ticks=0"
            % (self._oid,), on_restart=True)
        self._query_cmd = self._mcu.lookup_command(
            "query_closed_loop_stepper oid=%c clock=%u rest_ticks=%u",
            cq=self._cmd_queue)
        self._get_state_cmd = self._mcu.lookup_query_command(
            "closed_loop_stepper_get_state oid=%c",
            "closed_loop_stepper_state oid=%c corrected_steps=%i error_q16=%i",
            oid=self._oid, cq=self._cmd_queue)
        logging.info(
            "closed_loop_stepper %s: connected - stepper=%s sensor='angle %s'"
            " ratio_q16=%d deadband_q16=%d min_interval_ticks=%d",
            self.name, self._stepper_name, self._sensor_name,
            self._ratio_q16, self._deadband_q16, min_interval_ticks)

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

    def _status_poll_timer(self, eventtime):
        if self._enabled and self._get_state_cmd is not None:
            params = self._get_state_cmd.send([self._oid])
            self._corrected_steps = params['corrected_steps']
            self._error_steps = params['error_q16'] / 65536.
        return eventtime + STATUS_POLL_TIME

    # =========================================================================
    # GCode commands
    # =========================================================================

    def cmd_CLOSED_LOOP_STEPPER_STATUS(self, gcmd):
        if self._get_state_cmd is not None and self._enabled:
            params = self._get_state_cmd.send([self._oid])
            self._corrected_steps = params['corrected_steps']
            self._error_steps = params['error_q16'] / 65536.
        gcmd.respond_info(
            "CLOSED_LOOP_STEPPER [%s]  enabled=%-5s  corrected_steps=%-8s"
            "  error=%-10s"
            % (self.name, self._enabled, self._corrected_steps,
               "%.3f steps" % self._error_steps))

    def cmd_CLOSED_LOOP_STEPPER_ENABLE(self, gcmd):
        enable = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        self._send_enable(enable)
        gcmd.respond_info(
            "CLOSED_LOOP_STEPPER [%s]: %s"
            % (self.name, "enabled" if enable else "disabled"))

    # =========================================================================
    # Moonraker / status API
    # =========================================================================

    def get_status(self, eventtime=None):
        return {
            'enabled': self._enabled,
            'corrected_steps': self._corrected_steps,
            'error_steps': self._error_steps,
        }


def load_config_prefix(config):
    return ClosedLoopStepper(config)
