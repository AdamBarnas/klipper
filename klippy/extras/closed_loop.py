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
#   CLOSED_LOOP_STATUS  AXIS=x
#   CLOSED_LOOP_CORRECT AXIS=x
#   CLOSED_LOOP_ENABLE  AXIS=x ENABLE=0|1

import logging

# ===========================================================================
# PURE DOMAIN LOGIC
#
# No Klipper objects, printer references, or hardware access below this line.
# All values in mm, in commanded-position coordinate space.
# ===========================================================================

class ErrorTracker:
    """Tracks position error between angle-sensor readings and commanded position."""

    def __init__(self, correction_threshold, stall_threshold):
        if stall_threshold <= correction_threshold:
            raise ValueError("stall_threshold_mm must exceed correction_threshold_mm")
        self.correction_threshold = correction_threshold
        self.stall_threshold = stall_threshold

        self._sensor_pos = None     # latest calibrated reading from the angle sensor (mm)
        self._correction_count = 0
        self._stall_count = 0

    # ── Data ingestion ───────────────────────────────────────────────────────

    def update_sensor_position(self, pos_mm):
        self._sensor_pos = pos_mm

    # ── Queries ──────────────────────────────────────────────────────────────

    def has_data(self):
        return self._sensor_pos is not None

    def get_sensor_position(self):
        return self._sensor_pos

    def compute_error(self, commanded_mm):
        """Signed error (mm). Positive means the stepper is behind commanded position."""
        if self._sensor_pos is None:
            return None
        return commanded_mm - self._sensor_pos

    def needs_correction(self, commanded_mm):
        """Returns (bool, error_mm). True when |error| exceeds correction_threshold."""
        err = self.compute_error(commanded_mm)
        if err is None:
            return False, 0.
        return abs(err) > self.correction_threshold, err

    def is_stall(self, commanded_mm):
        """True when |error| exceeds stall_threshold."""
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

TOOLHEAD_IDLE_STATUS = 'Ready'
STALL_LOG_COOLDOWN   = 5.0    # seconds — prevents log spam for a persistent stall


class ClosedLoop:
    """Klipper extra: continuous closed-loop position correction for one axis."""

    def __init__(self, config):
        self.printer  = config.get_printer()
        self.name     = config.get_name().split()[-1]   # 'x' / 'y' from [closed_loop x]

        # ── Config ───────────────────────────────────────────────────────────
        axis = config.get('axis', self.name).lower()
        self._axis_idx       = {'x': 0, 'y': 1, 'z': 2}[axis]
        self._sensor_name    = config.get('sensor', '%s_angle_sensor' % axis)
        corr_threshold       = config.getfloat('correction_threshold_mm', 0.1, above=0.)
        stall_threshold      = config.getfloat('stall_threshold_mm',      2.0, above=0.)
        self._corr_speed     = config.getfloat('correction_speed',        20., above=0.)
        self._monitor_ivl    = config.getfloat('monitor_interval',        0.25, above=0.)
        self._enabled        = config.getboolean('enabled', True)

        # ── Domain logic (pure Python, no Klipper) ───────────────────────────
        self._tracker = ErrorTracker(corr_threshold, stall_threshold)

        # ── Klipper handles — populated at klippy:connect ────────────────────
        self._toolhead   = None
        self._gcode_move = None
        self._reactor    = self.printer.get_reactor()
        self._gcode      = self.printer.lookup_object('gcode')
        self._timer      = None

        # ── Internal state ───────────────────────────────────────────────────
        self._pending_correction = False   # prevents overlapping correction moves
        self._last_stall_time    = 0.

        # ── Klipper lifecycle ─────────────────────────────────────────────────
        self.printer.register_event_handler("klippy:connect",    self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

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

    # =========================================================================
    # Klipper lifecycle
    # =========================================================================

    def _handle_connect(self):
        self._toolhead   = self.printer.lookup_object('toolhead')
        self._gcode_move = self.printer.lookup_object('gcode_move')

        # Subscribe to the angle sensor's batch stream.
        # This also starts the sensor measurement if it isn't running yet.
        angle_obj = self.printer.lookup_object('angle %s' % self._sensor_name)
        angle_obj.add_client(self._angle_batch_handler)

        # Start the continuous monitoring timer.
        self._timer = self._reactor.register_timer(
            self._monitor_timer,
            self._reactor.monotonic() + self._monitor_ivl)

        logging.info("closed_loop %s: connected, monitoring 'angle %s'",
                     self.name, self._sensor_name)

    def _handle_disconnect(self):
        if self._timer is not None:
            self._reactor.unregister_timer(self._timer)
            self._timer = None

    # =========================================================================
    # Angle-sensor boundary
    #
    # Called by the angle module every ~100 ms with a batch of calibrated
    # samples.  The only Klipper object touched here is the batch message dict.
    # All business logic is delegated to ErrorTracker.
    # =========================================================================

    def _angle_batch_handler(self, msg):
        offset = msg.get('position_offset')     # calibrated position in mm, or None
        if offset is not None:
            self._tracker.update_sensor_position(offset)
        return True     # True = keep streaming; False = unsubscribe

    # =========================================================================
    # Reactor timer — continuous monitoring
    #
    # Runs every monitor_interval seconds in the Klipper reactor thread.
    # Must return the next wake time.  Must not block.
    # =========================================================================

    def _monitor_timer(self, eventtime):
        if not self._enabled or not self._tracker.has_data():
            return eventtime + self._monitor_ivl

        commanded = self._toolhead.get_position()[self._axis_idx]

        # ── Stall detection (checked regardless of motion state) ──────────────
        if self._tracker.is_stall(commanded):
            self._handle_stall(commanded, eventtime)
            return eventtime + self._monitor_ivl

        # ── Correction (only when toolhead is idle and no correction queued) ──
        if not self._pending_correction:
            th_status = self._toolhead.get_status(eventtime).get('status', '')
            if th_status == TOOLHEAD_IDLE_STATUS:
                needs, err = self._tracker.needs_correction(commanded)
                if needs:
                    self._pending_correction = True
                    # Defer the actual move one reactor iteration to avoid
                    # potential re-entrancy with the timer callback chain.
                    # (Pattern matches delayed_gcode's approach.)
                    self._reactor.register_callback(
                        lambda et: self._apply_correction_cb(commanded, err))

        return eventtime + self._monitor_ivl

    # =========================================================================
    # Stall handling
    # =========================================================================

    def _handle_stall(self, commanded, eventtime):
        if eventtime - self._last_stall_time < STALL_LOG_COOLDOWN:
            return
        self._last_stall_time = eventtime
        err = self._tracker.compute_error(commanded)
        self._tracker.record_stall()
        logging.warning(
            "closed_loop %s: stall detected — error=%.4f mm (total stalls: %d)",
            self.name, err, self._tracker.stall_count)
        # Use respond_info so the operator sees it immediately in the console.
        self._gcode.respond_info(
            "!! CLOSED_LOOP %s: stall detected! error=%.4f mm "
            "(total stalls: %d; use CLOSED_LOOP_CORRECT AXIS=%s to recover)"
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

    def _apply_correction_cb(self, original_commanded, original_err):
        """Reactor callback wrapper — re-validates before moving."""
        try:
            if not self._enabled or not self._tracker.has_data():
                return

            # Re-check: state may have changed since the timer queued us.
            th_status = self._toolhead.get_status(None).get('status', '')
            if th_status != TOOLHEAD_IDLE_STATUS:
                return

            commanded = self._toolhead.get_position()[self._axis_idx]
            needs, err = self._tracker.needs_correction(commanded)
            if not needs:
                return

            self._apply_correction(commanded, err)
        finally:
            self._pending_correction = False

    def _apply_correction(self, commanded_mm, error_mm):
        """Issue the correction move.  May be called from timer callback or gcode handler."""
        sensor_mm = self._tracker.get_sensor_position()

        # Build position vectors (full 4-axis: X Y Z E).
        current_pos = list(self._toolhead.get_position())

        # Where the axis actually is according to the sensor.
        actual_pos = list(current_pos)
        actual_pos[self._axis_idx] = sensor_mm

        # Where the axis should be (unchanged; this is the correction target).
        target_pos = list(current_pos)
        target_pos[self._axis_idx] = commanded_mm

        # Step 1: reconcile Klipper's internal position with sensor reality.
        self._toolhead.set_position(actual_pos)

        # Step 2: move back to the commanded position (executes the missing steps).
        self._toolhead.move(target_pos, self._corr_speed)

        # Step 3: sync the gcode coordinate layer so the user-visible position
        # remains consistent after the correction move.
        self._gcode_move.reset_last_position()

        self._tracker.record_correction()
        logging.info(
            "closed_loop %s: correction #%d — sensor=%.4f mm, commanded=%.4f mm, "
            "error=%.4f mm",
            self.name, self._tracker.correction_count,
            sensor_mm, commanded_mm, error_mm)

    # =========================================================================
    # GCode commands
    # =========================================================================

    def cmd_CLOSED_LOOP_STATUS(self, gcmd):
        commanded  = self._toolhead.get_position()[self._axis_idx]
        sensor_pos = self._tracker.get_sensor_position()
        err        = self._tracker.compute_error(commanded)
        gcmd.respond_info(
            "CLOSED_LOOP [%s]  enabled=%-5s  sensor=%-12s  commanded=%-12s  "
            "error=%-12s  corrections=%d  stalls=%d"
            % (self.name,
               self._enabled,
               "%.4f mm" % sensor_pos if sensor_pos is not None else "N/A",
               "%.4f mm" % commanded,
               "%.4f mm" % err        if err        is not None else "N/A",
               self._tracker.correction_count,
               self._tracker.stall_count))

    def cmd_CLOSED_LOOP_CORRECT(self, gcmd):
        """Force an immediate correction, waiting for any active moves to finish first."""
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

    # =========================================================================
    # Moonraker / status API
    # =========================================================================

    def get_status(self, eventtime=None):
        commanded = (self._toolhead.get_position()[self._axis_idx]
                     if self._toolhead else 0.)
        return {
            'enabled':         self._enabled,
            'sensor_position': self._tracker.get_sensor_position(),
            'error':           self._tracker.compute_error(commanded),
            'corrections':     self._tracker.correction_count,
            'stalls':          self._tracker.stall_count,
        }


def load_config_prefix(config):
    return ClosedLoop(config)
