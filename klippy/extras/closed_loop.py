# Closed-loop position correction using magnetic angle sensors.
#
# Copyright (C) 2025  Adam Barnas
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# ┌─────────────────────────────────────────────────────────────────────────┐
# │  ARCHITECTURE                                                           │
# │                                                                         │
# │  ErrorTracker  — pure Python, no Klipper imports.                      │
# │                  Owns all position-error business logic.               │
# │                                                                         │
# │  ClosedLoop    — Klipper interface only.                               │
# │                  All printer / toolhead / gcode / reactor calls here.  │
# └─────────────────────────────────────────────────────────────────────────┘
#
# Position tracking strategy
# ──────────────────────────
# The reference point is captured from the angle sensor immediately after the
# axis is homed (stepper:sync_mcu_position event → next angle batch).
# Subsequent sensor positions are computed as:
#
#   sensor_pos_mm = home_pos_mm + (current_angle - home_angle) * mm_per_count
#
# where current_angle is the calibrated, accumulated angle from the angle
# module's batch stream (msg['data'][-1][1]).  Using the accumulated value
# (which grows beyond 65535 across revolutions) means no special wraparound
# handling is needed.  The calibration lookup in the angle module already
# linearises the sensor and flips direction, so the angle always increases
# with increasing commanded position.
#
# Config:
#   [closed_loop x]
#   sensor: x_angle_sensor          # name of the [angle] section (no "angle " prefix)
#   correction_threshold_mm: 0.1    # minimum error that triggers a correction move
#   stall_threshold_mm: 2.0         # error that triggers a stall warning
#   correction_speed: 20            # mm/s for correction moves
#   monitor_interval: 0.250         # seconds between automatic checks
#   enabled: True
#
# GCode commands (all take AXIS=<name> to select the instance):
#   CLOSED_LOOP_STATUS        AXIS=x
#   CLOSED_LOOP_CORRECT       AXIS=x
#   CLOSED_LOOP_ENABLE        AXIS=x ENABLE=0|1
#   CLOSED_LOOP_AUTO_CORRECT  AXIS=x ENABLE=0|1

import logging

# ===========================================================================
# PURE DOMAIN LOGIC
#
# No Klipper objects, printer references, or hardware access below this line.
# All values in mm / angle counts.
# ===========================================================================

class ErrorTracker:
    """Tracks position error between the angle sensor and the commanded position.

    Reference is established at homing via set_home_reference().
    All subsequent sensor positions are derived from the accumulated angle
    delta since that moment.
    """

    def __init__(self, correction_threshold, stall_threshold, mm_per_count):
        if stall_threshold <= correction_threshold:
            raise ValueError("stall_threshold_mm must exceed correction_threshold_mm")
        self.correction_threshold = correction_threshold
        self.stall_threshold      = stall_threshold
        self._mm_per_count        = mm_per_count   # rotation_distance / 65536

        # Homing reference — set by set_home_reference(), cleared on klippy:connect
        self._is_homed     = False
        self._home_angle   = None   # calibrated accumulated angle at homing
        self._home_pos_mm  = None   # commanded position at homing (usually 0)

        # Live state — updated by update_angle() on every batch
        self._current_angle = None

        self._correction_count = 0
        self._stall_count      = 0

    # ── Reference capture ────────────────────────────────────────────────────

    def set_home_reference(self, calibrated_angle, commanded_pos_mm):
        """Establish the sensor zero-point.  Must be called after each homing."""
        self._home_angle    = calibrated_angle
        self._home_pos_mm   = commanded_pos_mm
        self._current_angle = calibrated_angle
        self._is_homed      = True

    # ── Live update ──────────────────────────────────────────────────────────

    def update_angle(self, calibrated_angle):
        """Ingest the latest calibrated accumulated angle from the batch stream."""
        self._current_angle = calibrated_angle

    # ── Queries ──────────────────────────────────────────────────────────────

    def has_data(self):
        """True only after the axis has been homed at least once."""
        return self._is_homed and self._current_angle is not None

    def get_sensor_position(self):
        """Current sensor-derived position in mm, or None if not homed."""
        if not self.has_data():
            return None
        return (self._home_pos_mm
                + (self._current_angle - self._home_angle) * self._mm_per_count)

    def compute_error(self, commanded_mm):
        """Signed error (mm).  Positive = stepper is behind commanded position."""
        sensor = self.get_sensor_position()
        if sensor is None:
            return None
        return commanded_mm - sensor

    def needs_correction(self, commanded_mm):
        """Returns (bool, error_mm).  True when |error| > correction_threshold."""
        err = self.compute_error(commanded_mm)
        if err is None:
            return False, 0.
        return abs(err) > self.correction_threshold, err

    def is_stall(self, commanded_mm):
        """True when |error| > stall_threshold."""
        err = self.compute_error(commanded_mm)
        return err is not None and abs(err) > self.stall_threshold

    # ── Counters ─────────────────────────────────────────────────────────────

    def record_correction(self):
        self._correction_count += 1

    def record_stall(self):
        self._stall_count += 1

    @property
    def correction_count(self):
        return self._correction_count

    @property
    def stall_count(self):
        return self._stall_count


# ===========================================================================
# KLIPPER INTERFACE
#
# ErrorTracker is the only object from the section above used here.
# Everything else (printer, toolhead, gcode, reactor) lives in this section.
# ===========================================================================

STALL_LOG_COOLDOWN = 5.0    # seconds — prevents log spam for a persistent stall


class ClosedLoop:
    """Klipper extra: continuous closed-loop position correction for one axis."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name    = config.get_name().split()[-1]   # 'x' / 'y' from [closed_loop x]

        # ── Config ───────────────────────────────────────────────────────────
        axis = config.get('axis', self.name).lower()
        self._axis_idx       = {'x': 0, 'y': 1, 'z': 2}[axis]
        self._stepper_name   = 'stepper_%s' % axis   # matched against sync_mcu_position
        self._sensor_name    = config.get('sensor', '%s_angle_sensor' % axis)
        self._corr_threshold      = config.getfloat('correction_threshold_mm', 0.1,  above=0.)
        self._stall_threshold     = config.getfloat('stall_threshold_mm',      2.0,  above=0.)
        self._corr_speed          = config.getfloat('correction_speed',        20.,  above=0.)
        self._monitor_ivl         = config.getfloat('monitor_interval',        0.25, above=0.)
        self._enabled             = config.getboolean('enabled', True)
        self._auto_correct_stall  = config.getboolean('auto_correct_on_stall', False)

        # ErrorTracker is built in _handle_connect once mm_per_count is known.
        self._tracker = None

        # ── Klipper handles — populated at klippy:connect ────────────────────
        self._toolhead   = None
        self._gcode_move = None
        self._reactor    = self.printer.get_reactor()
        self._gcode      = self.printer.lookup_object('gcode')
        self._timer      = None

        # ── Internal state ───────────────────────────────────────────────────
        self._pending_home_capture  = False  # set by sync event, cleared by batch handler
        self._pending_correction    = False  # prevents overlapping correction moves
        self._stall_correct_pending = False  # stall detected, correct on next idle tick
        self._last_move_speed       = self._corr_speed  # updated every timer tick
        self._last_stall_time       = 0.
        self._step_enable           = None   # populated at connect

        # ── Klipper lifecycle ─────────────────────────────────────────────────
        self.printer.register_event_handler("klippy:connect",    self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        # stepper:sync_mcu_position fires whenever Klipper resets a stepper's
        # position — this happens at the end of every homing move.
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self._handle_sync_mcu_pos)

        # ── GCode commands ────────────────────────────────────────────────────
        self._gcode.register_mux_command(
            'CLOSED_LOOP_STATUS', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_STATUS,
            desc="Report closed-loop position status for an axis")
        self._gcode.register_mux_command(
            'CLOSED_LOOP_CORRECT', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_CORRECT,
            desc="Force an immediate closed-loop correction on an axis")
        self._gcode.register_mux_command(
            'CLOSED_LOOP_ENABLE', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_ENABLE,
            desc="Enable or disable closed-loop monitoring (ENABLE=0|1)")
        self._gcode.register_mux_command(
            'CLOSED_LOOP_AUTO_CORRECT', 'AXIS', self.name,
            self.cmd_CLOSED_LOOP_AUTO_CORRECT,
            desc="Enable or disable auto-correction on stall detection (ENABLE=0|1)")

    # =========================================================================
    # Klipper lifecycle
    # =========================================================================

    def _handle_connect(self):
        self._toolhead   = self.printer.lookup_object('toolhead')
        self._gcode_move = self.printer.lookup_object('gcode_move')

        mm_per_count  = self._lookup_mm_per_count()
        self._tracker = ErrorTracker(self._corr_threshold, self._stall_threshold,
                                     mm_per_count)

        stepper_enable   = self.printer.lookup_object('stepper_enable')
        self._step_enable = stepper_enable.lookup_enable(self._stepper_name)

        angle_obj = self.printer.lookup_object('angle %s' % self._sensor_name)
        angle_obj.add_client(self._angle_batch_handler)

        self._timer = self._reactor.register_timer(
            self._monitor_timer,
            self._reactor.monotonic() + self._monitor_ivl)

        logging.info(
            "closed_loop %s: connected — stepper=%s  sensor='angle %s'  "
            "mm_per_count=%.8f  (home the axis to activate)",
            self.name, self._stepper_name, self._sensor_name, mm_per_count)

    def _handle_disconnect(self):
        if self._timer is not None:
            self._reactor.unregister_timer(self._timer)
            self._timer = None

    def _lookup_mm_per_count(self):
        """Compute mm per 16-bit angle count from the stepper's rotation_distance."""
        configfile = self.printer.lookup_object('configfile')
        settings   = configfile.get_status(None)['settings']
        stconfig   = settings.get(self._stepper_name, {})
        rotation_distance = stconfig.get('rotation_distance', 40.0)
        # The angle module scales all sensors to 16-bit (65536 counts/revolution).
        return rotation_distance / 65536.0

    # =========================================================================
    # Homing event boundary
    #
    # stepper:sync_mcu_position is fired by Klipper at the end of every homing
    # move, passing the mcu_stepper object whose position was just reset.
    # We set a flag so the very next angle batch is used as the home reference.
    # =========================================================================

    def _handle_sync_mcu_pos(self, mcu_stepper):
        if mcu_stepper.get_name() == self._stepper_name:
            self._pending_home_capture = True
            logging.info(
                "closed_loop %s: homing detected for %s — "
                "will capture home reference on next angle batch",
                self.name, self._stepper_name)

    # =========================================================================
    # Angle-sensor boundary
    #
    # Called by the angle module every ~100 ms with a batch of calibrated
    # samples.  msg['data'] is a list of (time, calibrated_accumulated_angle)
    # tuples.  The angles are already linearised and direction-corrected by
    # apply_calibration() in the angle module — they increase monotonically
    # with increasing commanded position regardless of sensor orientation.
    # =========================================================================

    def _angle_batch_handler(self, msg):
        samples = msg.get('data', [])
        if not samples:
            return True

        _, latest_angle = samples[-1]

        if self._pending_home_capture and self._toolhead is not None:
            # Axis was just homed.  Commanded position is now the endstop value
            # (typically 0 for X, position_max for Y with homing_positive_dir).
            commanded = self._toolhead.get_position()[self._axis_idx]
            self._tracker.set_home_reference(latest_angle, commanded)
            self._pending_home_capture = False
            logging.info(
                "closed_loop %s: home reference set — "
                "angle=%.1f  commanded=%.4f mm",
                self.name, latest_angle, commanded)
        else:
            self._tracker.update_angle(latest_angle)

        return True     # True = keep streaming; False = unsubscribe

    # =========================================================================
    # Reactor timer — continuous monitoring
    #
    # Runs every monitor_interval seconds in the Klipper reactor thread.
    # Must return the next wake time.  Must not block.
    # =========================================================================

    def _monitor_timer(self, eventtime):
        if not self._enabled or not self._tracker or not self._tracker.has_data():
            return eventtime + self._monitor_ivl

        # Track last move speed every tick so it reflects the most recent move.
        self._last_move_speed = self._gcode_move.speed

        commanded = self._toolhead.get_position()[self._axis_idx]

        # ── Stall detection (checked regardless of motion state) ──────────────
        if self._tracker.is_stall(commanded):
            self._handle_stall(commanded, eventtime)
            # Don't return early — still fall through to the idle correction
            # check so a stall-triggered correction is applied as soon as the
            # toolhead stops moving.

        # ── Idle corrections (normal drift and pending stall corrections) ──────
        if not self._pending_correction:
            print_time, est_print_time, lookahead_empty = \
                self._toolhead.check_busy(eventtime)
            if lookahead_empty and est_print_time > print_time:
                if self._stall_correct_pending:
                    # Stall correction takes priority; use speed of the last move.
                    self._stall_correct_pending = False
                    self._pending_correction    = True
                    speed = self._last_move_speed
                    self._reactor.register_callback(
                        lambda et: self._apply_correction_cb(commanded, speed))
                else:
                    needs, err = self._tracker.needs_correction(commanded)
                    if needs:
                        self._pending_correction = True
                        self._reactor.register_callback(
                            lambda et: self._apply_correction_cb(
                                commanded, self._corr_speed))

        return eventtime + self._monitor_ivl

    # =========================================================================
    # Stall handling
    # =========================================================================

    def _handle_stall(self, commanded, eventtime):
        motors_on = self._step_enable is not None and self._step_enable.is_motor_enabled()

        # Queue auto-correction when conditions are met.
        # The actual move is deferred to the next idle tick via _stall_correct_pending.
        if self._auto_correct_stall and motors_on:
            self._stall_correct_pending = True

        # Log at most once per cooldown period to avoid console spam.
        if eventtime - self._last_stall_time < STALL_LOG_COOLDOWN:
            return
        self._last_stall_time = eventtime
        err = self._tracker.compute_error(commanded)
        self._tracker.record_stall()
        logging.warning(
            "closed_loop %s: stall detected — error=%.4f mm (total stalls: %d)",
            self.name, err, self._tracker.stall_count)

        if self._auto_correct_stall and motors_on:
            self._gcode.respond_info(
                "!! CLOSED_LOOP %s: stall detected! error=%.4f mm — "
                "auto-correction queued (total stalls: %d)"
                % (self.name.upper(), err, self._tracker.stall_count))
        else:
            self._gcode.respond_info(
                "!! CLOSED_LOOP %s: stall detected! error=%.4f mm "
                "(total stalls: %d; home the axis or use CLOSED_LOOP_CORRECT AXIS=%s)"
                % (self.name.upper(), err, self._tracker.stall_count, self.name))

    # =========================================================================
    # Correction move
    #
    # Strategy:
    #   1. Tell Klipper where the axis actually is (sensor reading).
    #   2. Move back to the originally commanded position.
    #   3. Sync the gcode coordinate state so subsequent moves are not offset.
    #
    # This causes the steppers to execute the missing steps without disturbing
    # the logical coordinate system seen by the rest of the print.
    # =========================================================================

    def _apply_correction_cb(self, original_commanded, speed):
        """Reactor callback wrapper — re-validates before moving."""
        try:
            if not self._enabled or not self._tracker or not self._tracker.has_data():
                return
            eventtime = self._reactor.monotonic()
            print_time, est_print_time, lookahead_empty = \
                self._toolhead.check_busy(eventtime)
            if not lookahead_empty or est_print_time <= print_time:
                return
            commanded = self._toolhead.get_position()[self._axis_idx]
            needs, err = self._tracker.needs_correction(commanded)
            if not needs:
                return
            self._apply_correction(commanded, err, speed)
        finally:
            self._pending_correction = False

    def _apply_correction(self, commanded_mm, error_mm, speed=None):
        """Issue the correction move.  May be called from reactor callback or gcode."""
        if speed is None:
            speed = self._corr_speed
        sensor_mm   = self._tracker.get_sensor_position()
        current_pos = list(self._toolhead.get_position())

        actual_pos = list(current_pos)
        actual_pos[self._axis_idx] = sensor_mm        # where we actually are

        target_pos = list(current_pos)
        target_pos[self._axis_idx] = commanded_mm     # where we need to be

        # Step 1: reconcile Klipper's internal position with sensor reality.
        self._toolhead.set_position(actual_pos)

        # Step 2: move back to the commanded position (executes the missing steps).
        self._toolhead.move(target_pos, speed)

        # Step 3: sync the gcode coordinate layer so user-visible position stays consistent.
        self._gcode_move.reset_last_position()

        self._tracker.record_correction()
        logging.info(
            "closed_loop %s: correction #%d — sensor=%.4f mm  commanded=%.4f mm  "
            "error=%.4f mm  speed=%.1f mm/s",
            self.name, self._tracker.correction_count,
            sensor_mm, commanded_mm, error_mm, speed)

    # =========================================================================
    # GCode commands
    # =========================================================================

    def cmd_CLOSED_LOOP_STATUS(self, gcmd):
        if not self._tracker or not self._tracker.has_data():
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: not homed — home the axis to activate"
                % self.name)
            return
        commanded  = self._toolhead.get_position()[self._axis_idx]
        sensor_pos = self._tracker.get_sensor_position()
        err        = self._tracker.compute_error(commanded)
        gcmd.respond_info(
            "CLOSED_LOOP [%s]  enabled=%-5s  sensor=%-12s  commanded=%-12s  "
            "error=%-12s  corrections=%d  stalls=%d"
            % (self.name,
               self._enabled,
               "%.4f mm" % sensor_pos,
               "%.4f mm" % commanded,
               "%.4f mm" % err,
               self._tracker.correction_count,
               self._tracker.stall_count))

    def cmd_CLOSED_LOOP_CORRECT(self, gcmd):
        """Force an immediate correction, waiting for any active moves to finish first."""
        if not self._tracker or not self._tracker.has_data():
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: axis not homed — cannot correct" % self.name)
            return
        self._toolhead.wait_moves()
        commanded = self._toolhead.get_position()[self._axis_idx]
        needs, err = self._tracker.needs_correction(commanded)
        if needs:
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: applying correction, error=%.4f mm"
                % (self.name, err))
            self._apply_correction(commanded, err)
        else:
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: no correction needed (error=%.4f mm)"
                % (self.name, err if err is not None else 0.))

    def cmd_CLOSED_LOOP_ENABLE(self, gcmd):
        self._enabled = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        gcmd.respond_info(
            "CLOSED_LOOP [%s]: %s"
            % (self.name, "enabled" if self._enabled else "disabled"))

    def cmd_CLOSED_LOOP_AUTO_CORRECT(self, gcmd):
        """Toggle automatic correction on stall detection."""
        self._auto_correct_stall = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        if self._auto_correct_stall:
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: auto-correct on stall ENABLED "
                "(uses speed of last move)" % self.name)
        else:
            gcmd.respond_info(
                "CLOSED_LOOP [%s]: auto-correct on stall DISABLED" % self.name)

    # =========================================================================
    # Moonraker / status API
    # =========================================================================

    def get_status(self, eventtime=None):
        commanded = (self._toolhead.get_position()[self._axis_idx]
                     if self._toolhead else 0.)
        tracker = self._tracker
        return {
            'enabled':              self._enabled,
            'auto_correct_on_stall': self._auto_correct_stall,
            'is_homed':             tracker.has_data()             if tracker else False,
            'sensor_position':      tracker.get_sensor_position()  if tracker else None,
            'error':                tracker.compute_error(commanded) if tracker else None,
            'corrections':          tracker.correction_count       if tracker else 0,
            'stalls':               tracker.stall_count            if tracker else 0,
        }


def load_config_prefix(config):
    return ClosedLoop(config)
