# A utility to run a smooth, broadband frequency-swept (chirp) excitation
# for system identification.
#
# TEST_RESONANCES (resonance_tester.py) sweeps frequency using a BANG-BANG
# (square-wave) acceleration profile: constant +accel for half a period,
# then constant -accel for the other half. That's fine for input-shaper
# tuning (it only cares about the fundamental), but a square wave has
# large odd harmonics (3rd harmonic at 1/3 amplitude, 5th at 1/5, ...) -
# while sweeping at 40 Hz it is ALSO exciting 120 Hz and 200 Hz, which can
# contaminate a system-identification fit with energy that didn't
# actually come from the plant's response at those frequencies.
#
# TEST_CHIRP instead samples-and-holds a genuine sine wave at several
# points per cycle (SEGMENTS_PER_CYCLE, default 8), which pushes the
# first stray harmonics out to roughly (SEGMENTS_PER_CYCLE-1) times the
# fundamental at a much smaller amplitude - a much cleaner broadband
# excitation for MATLAB/System Identification Toolbox-style fitting.
#
# This module deliberately does NOT duplicate resonance_tester.py's
# accelerometer wiring, axis parsing, or move-safety logic (velocity/
# accel clamping, input-shaper disable/enable, Z-axis limit checks) - it
# reuses the already-configured [resonance_tester] object for all of
# that, and only supplies a different excitation waveform.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, time
from . import resonance_tester

class ChirpTestGenerator:
    def __init__(self, config):
        self.min_freq = config.getfloat('min_freq', 5., minval=0.1)
        self.max_freq = config.getfloat('max_freq', 200.,
                                        minval=self.min_freq, maxval=300.)
        self.max_freq_z = config.getfloat('max_freq_z', 100.,
                                          minval=self.min_freq, maxval=300.)
        self.accel_per_hz = config.getfloat('accel_per_hz', 60., above=0.)
        self.accel_per_hz_z = config.getfloat('accel_per_hz_z', 15.,
                                              above=0.)
        self.duration = config.getfloat('duration', 60., above=1.)
        self.sweep_type = config.get('sweep_type', 'log').lower()
        if self.sweep_type not in ('log', 'linear'):
            raise config.error(
                    "chirp_tester: sweep_type must be 'log' or 'linear'")
        self.segments_per_cycle = config.getint('segments_per_cycle', 8,
                                                 minval=4, maxval=32)
    def prepare_test(self, gcmd, is_z):
        self.freq_start = gcmd.get_float("FREQ_START", self.min_freq,
                                         minval=0.1)
        self.freq_end = gcmd.get_float(
                "FREQ_END", (self.max_freq_z if is_z else self.max_freq),
                minval=self.freq_start, maxval=300.)
        self.test_accel_per_hz = gcmd.get_float(
                "ACCEL_PER_HZ",
                (self.accel_per_hz_z if is_z else self.accel_per_hz),
                above=0.)
        self.test_duration = gcmd.get_float("DURATION", self.duration,
                                            above=1.)
        self.test_sweep_type = gcmd.get("SWEEP_TYPE",
                                        self.sweep_type).lower()
        if self.test_sweep_type not in ('log', 'linear'):
            raise gcmd.error("SWEEP_TYPE must be LOG or LINEAR")
        self.test_segments_per_cycle = gcmd.get_int(
                "SEGMENTS_PER_CYCLE", self.segments_per_cycle,
                minval=4, maxval=32)
    def gen_test(self):
        # Sample-and-hold sine: within each cycle at the current
        # instantaneous frequency, hold N_SEG constant-acceleration
        # segments at successive points around the sine, one full period
        # per pass through the inner loop. Frequency is advanced once per
        # cycle - the same "constant within a short segment" approximation
        # resonance_tester.py's own generator already relies on, just
        # subdivided finer to approximate a sine instead of a square wave.
        freq = self.freq_start
        n_seg = self.test_segments_per_cycle
        res = []
        t = 0.
        two_pi = 2. * math.pi
        if self.test_sweep_type == 'log':
            log_rate = (math.log(self.freq_end / self.freq_start)
                        / self.test_duration)
        else:
            hz_per_sec = (self.freq_end - self.freq_start) \
                    / self.test_duration
        while freq <= self.freq_end + 0.000001:
            t_seg = 1. / (n_seg * freq)
            accel_amp = self.test_accel_per_hz * freq
            for i in range(n_seg):
                t += t_seg
                phase = two_pi * (i + 1) / n_seg
                res.append((t, accel_amp * math.sin(phase), freq))
            t_cycle = n_seg * t_seg
            if self.test_sweep_type == 'log':
                freq *= math.exp(log_rate * t_cycle)
            else:
                freq += hz_per_sec * t_cycle
        return res
    def get_max_freq(self):
        return self.freq_end

def _peak_abs_displacement(test_seq):
    # Offline replay of the same (time, accel, freq) -> position
    # integration ResonanceTestExecutor.run_test performs move-by-move,
    # just to find the largest excursion from the start position before
    # any moves are actually issued (used to auto-scale the sweep to fit
    # the requested X/Y travel window - see cmd_TEST_CHIRP below).
    last_v = last_t = 0.
    pos = 0.
    peak = 0.
    for next_t, accel, freq in test_seq:
        t_seg = next_t - last_t
        if abs(accel) < 0.000001:
            d = last_v * t_seg
        else:
            v = last_v + accel * t_seg
            d = (v * v - last_v * last_v) * (.5 / accel)
            last_v = v
        pos += d
        peak = max(peak, abs(pos))
        last_t = next_t
    return peak

class ChirpTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.generator = ChirpTestGenerator(config)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("TEST_CHIRP", self.cmd_TEST_CHIRP,
                                    desc=self.cmd_TEST_CHIRP_help)
        self.printer.register_event_handler("klippy:connect", self.connect)
    def connect(self):
        # Reuse [resonance_tester]'s already-configured accelerometer
        # chips, probe points, move-safety executor and filename/CSV
        # helpers - see the module docstring for why.
        self.resonance_tester = self.printer.lookup_object('resonance_tester')

    cmd_TEST_CHIRP_help = (
        "Runs a smooth broadband frequency-swept (chirp) excitation for"
        " system identification")
    def cmd_TEST_CHIRP(self, gcmd):
        rt = self.resonance_tester
        toolhead = self.printer.lookup_object('toolhead')

        axis = resonance_tester._parse_axis(gcmd, gcmd.get("AXIS").lower())

        # Keep the whole test inside a safe X/Y travel window: center the
        # test on the middle of the window (rather than resonance_tester's
        # probe_points, which can sit right on a boundary) and, once the
        # sweep is generated below, auto-scale its amplitude down if the
        # projected excursion would otherwise leave the window.
        x_min = gcmd.get_float("X_MIN", 100.)
        x_max = gcmd.get_float("X_MAX", 200., minval=x_min)
        y_min = gcmd.get_float("Y_MIN", 100.)
        y_max = gcmd.get_float("Y_MAX", 200., minval=y_min)
        tpos = toolhead.get_position()
        test_point = [(x_min + x_max) / 2., (y_min + y_max) / 2., tpos[2]]

        chips_str = gcmd.get("CHIPS", None)
        accel_chips = rt._parse_chips(chips_str) if chips_str else None

        outputs = gcmd.get("OUTPUT", "raw_data").lower().split(',')
        for output in outputs:
            if output not in ('resonances', 'raw_data'):
                raise gcmd.error(
                        "Unsupported output '%s', only 'resonances' and"
                        " 'raw_data' are supported" % (output,))
        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        if not rt.is_valid_name_suffix(name_suffix):
            raise gcmd.error("Invalid NAME parameter")
        csv_output = 'resonances' in outputs
        raw_output = 'raw_data' in outputs

        helper = None
        if csv_output:
            from . import shaper_calibrate
            helper = shaper_calibrate.ShaperCalibrate(self.printer)

        is_z = bool(axis.get_dir()[2])
        self.generator.prepare_test(gcmd, is_z)
        test_seq = self.generator.gen_test()
        gcmd.respond_info(
                "Chirp: %.1f Hz -> %.1f Hz over %.1f s (%s sweep, %d"
                " samples/cycle, %d moves)"
                % (self.generator.freq_start, self.generator.freq_end,
                   self.generator.test_duration,
                   self.generator.test_sweep_type,
                   self.generator.test_segments_per_cycle, len(test_seq)))
        if len(test_seq) > 200000:
            gcmd.respond_info(
                    "WARNING: this generates %d moves and may be slow to"
                    " plan - consider a shorter DURATION, higher"
                    " FREQ_START, or fewer SEGMENTS_PER_CYCLE"
                    % len(test_seq))

        # Scale the sweep down (uniformly - the system is linear in accel
        # amplitude, so this doesn't distort the frequency content) if its
        # projected excursion would carry X or Y outside the requested
        # window. dir_x/dir_y are the vibration direction's components
        # along X/Y; a pure AXIS=x or AXIS=y test only has one of them
        # non-zero, so the other axis just stays fixed at the window's
        # center and is trivially within range.
        dir_x, dir_y = axis.get_dir()[0], axis.get_dir()[1]
        half_ranges = []
        if abs(dir_x) > 0.000001:
            half_ranges.append(((x_max - x_min) / 2.) / abs(dir_x))
        if abs(dir_y) > 0.000001:
            half_ranges.append(((y_max - y_min) / 2.) / abs(dir_y))
        if half_ranges:
            half_range = min(half_ranges)
            peak = _peak_abs_displacement(test_seq)
            if peak > half_range:
                scale = half_range / peak
                test_seq = [(t, a * scale, f) for t, a, f in test_seq]
                gcmd.respond_info(
                        "Chirp amplitude scaled by %.3fx (peak excursion"
                        " was %.1f mm) to keep the sweep within"
                        " X=[%.0f,%.0f] Y=[%.0f,%.0f]"
                        % (scale, peak, x_min, x_max, y_min, y_max))

        if test_point:
            toolhead.manual_move(test_point, rt.move_speed)
        toolhead.wait_moves()
        toolhead.dwell(0.500)

        raw_values = []
        if accel_chips is None:
            for chip_axis, chip in rt.accel_chips:
                if axis.matches(chip_axis):
                    aclient = chip.start_internal_client()
                    raw_values.append((chip_axis, aclient, chip.name))
        else:
            for chip in accel_chips:
                aclient = chip.start_internal_client()
                raw_values.append((axis, aclient, chip.name))
        if not raw_values:
            raise gcmd.error(
                    "No accelerometers specified that can measure"
                    " resonances over axis '%s'" % axis.get_name())

        # All move generation, velocity/accel safety clamping, Z-axis
        # limit checks and input-shaper disable/enable is handled by the
        # shared executor - identical safety behavior to TEST_RESONANCES.
        rt.executor.run_test(test_seq, axis, gcmd)

        for chip_axis, aclient, chip_name in raw_values:
            aclient.finish_measurements()
            if raw_output:
                raw_name = rt.get_filename(
                        'raw_data', name_suffix, axis,
                        chip_name=chip_name if len(raw_values) > 1 else None)
                aclient.write_to_file(raw_name)
                gcmd.respond_info(
                        "Writing raw accelerometer data to %s file"
                        % (raw_name,))
        if csv_output:
            calibration_data = None
            for chip_axis, aclient, chip_name in raw_values:
                if not aclient.has_valid_samples():
                    raise gcmd.error(
                            "accelerometer '%s' measured no data"
                            % (chip_name,))
                name = rt.get_filename(
                        'resonances', name_suffix, axis,
                        chip_name=chip_name if len(raw_values) > 1 else None)
                new_data = helper.process_accelerometer_data(name, aclient)
                if calibration_data is None:
                    calibration_data = new_data
                else:
                    calibration_data.add_data(new_data)
            csv_name = rt.save_calibration_data(
                    'resonances', name_suffix, helper, axis,
                    calibration_data, max_freq=1.5 * self.generator.get_max_freq())
            gcmd.respond_info(
                    "Resonances data written to %s file" % (csv_name,))

def load_config(config):
    return ChirpTester(config)
