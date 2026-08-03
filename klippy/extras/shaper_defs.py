# Definitions of the supported input shapers
#
# Copyright (C) 2020-2021  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, math

SHAPER_VIBRATION_REDUCTION=20.
DEFAULT_DAMPING_RATIO = 0.1

InputShaperCfg = collections.namedtuple(
        'InputShaperCfg',
        ('name', 'init_func', 'min_freq', 'max_damping_ratio'))

def get_none_shaper():
    return ([], [])

def get_zv_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = 1. / (shaper_freq * df)
    A = [1., K]
    T = [0., .5*t_d]
    return (A, T)

def get_zvd_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = 1. / (shaper_freq * df)
    A = [1., 2.*K, K**2]
    T = [0., .5*t_d, t_d]
    return (A, T)

def get_mzv_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-.75 * damping_ratio * math.pi / df)
    t_d = 1. / (shaper_freq * df)

    a1 = 1. - 1. / math.sqrt(2.)
    a2 = (math.sqrt(2.) - 1.) * K
    a3 = a1 * K * K

    A = [a1, a2, a3]
    T = [0., .375*t_d, .75*t_d]
    return (A, T)

def get_ei_shaper(shaper_freq, damping_ratio):
    v_tol = 1. / SHAPER_VIBRATION_REDUCTION # vibration tolerance
    df = math.sqrt(1. - damping_ratio**2)
    t_d = 1. / (shaper_freq * df)
    dr = damping_ratio

    a1 = (0.24968 + 0.24961 * v_tol) + (( 0.80008 + 1.23328 * v_tol) +
                                        ( 0.49599 + 3.17316 * v_tol) * dr) * dr
    a3 = (0.25149 + 0.21474 * v_tol) + ((-0.83249 + 1.41498 * v_tol) +
                                        ( 0.85181 - 4.90094 * v_tol) * dr) * dr
    a2 = 1. - a1 - a3

    t2 = 0.4999 + ((( 0.46159 + 8.57843 * v_tol) * v_tol) +
                   (((4.26169 - 108.644 * v_tol) * v_tol) +
                    ((1.75601 + 336.989 * v_tol) * v_tol) * dr) * dr) * dr

    A = [a1, a2, a3]
    T = [0., t2 * t_d, t_d]
    return (A, T)

def _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a):
    tau = 1. / shaper_freq
    T = []
    A = []
    n = len(a)
    k = len(a[0])
    for i in range(n):
        u = t[i][k-1]
        v = a[i][k-1]
        for j in range(k-1):
            u = u * damping_ratio + t[i][k-j-2]
            v = v * damping_ratio + a[i][k-j-2]
        T.append(u * tau)
        A.append(v)
    return (A, T)

def get_2hump_ei_shaper(shaper_freq, damping_ratio):
    t = [[0., 0., 0., 0.],
         [0.49890,  0.16270, -0.54262, 6.16180],
         [0.99748,  0.18382, -1.58270, 8.17120],
         [1.49920, -0.09297, -0.28338, 1.85710]]
    a = [[0.16054,  0.76699,  2.26560, -1.22750],
         [0.33911,  0.45081, -2.58080,  1.73650],
         [0.34089, -0.61533, -0.68765,  0.42261],
         [0.15997, -0.60246,  1.00280, -0.93145]]
    return _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a)

def get_3hump_ei_shaper(shaper_freq, damping_ratio):
    t = [[0., 0., 0., 0.],
         [0.49974,  0.23834,  0.44559, 12.4720],
         [0.99849,  0.29808, -2.36460, 23.3990],
         [1.49870,  0.10306, -2.01390, 17.0320],
         [1.99960, -0.28231,  0.61536, 5.40450]]
    a = [[0.11275,  0.76632,  3.29160, -1.44380],
         [0.23698,  0.61164, -2.57850,  4.85220],
         [0.30008, -0.19062, -2.14560,  0.13744],
         [0.23775, -0.73297,  0.46885, -2.08650],
         [0.11244, -0.45439,  0.96382, -1.46000]]
    return _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a)

# min_freq for each shaper is chosen to have projected max_accel ~= 1500
INPUT_SHAPERS = [
    InputShaperCfg(name='zv', init_func=get_zv_shaper,
                   min_freq=21., max_damping_ratio=0.99),
    InputShaperCfg(name='mzv', init_func=get_mzv_shaper,
                   min_freq=23., max_damping_ratio=0.99),
    InputShaperCfg(name='zvd', init_func=get_zvd_shaper,
                   min_freq=29., max_damping_ratio=0.99),
    InputShaperCfg(name='ei', init_func=get_ei_shaper,
                   min_freq=29., max_damping_ratio=0.4),
    InputShaperCfg(name='2hump_ei', init_func=get_2hump_ei_shaper,
                   min_freq=39., max_damping_ratio=0.3),
    InputShaperCfg(name='3hump_ei', init_func=get_3hump_ei_shaper,
                   min_freq=48., max_damping_ratio=0.2),
]

######################################################################
# Multi-mode (4th-order+) input shapers
#
# A single-mode shaper above places one pair of zeros in the vibration
# transfer function (eq:vibration_transfer in Klipper_input_shaping.tex),
# tuned to cancel exactly one resonant mode (f_n, damping_ratio). Real
# axes are not always well described by a single second-order oscillator
# -- e.g. an axis driven by two independently-coupled motors can show two
# separate resonant modes, making it effectively a 4th-order system (two
# independent complex pole pairs; see the empirical fits in
# Klipper_input_shaping.tex section 2.7).
#
# Convolving two impulse sequences multiplies their vibration-transfer
# functions: A_{S1*S2}(w) = A_S1(w) * A_S2(w). If S1 is built to zero
# vibrations exactly at (f1, zeta1), that zero survives in the product
# regardless of what S2 does there (one factor is zero), and
# symmetrically for S2 at (f2, zeta2). So convolving a shaper tuned to
# mode 1 with one tuned to mode 2 cancels both modes simultaneously.
# This is the standard "multi-mode input shaping" construction (Singer &
# Seering, 1990; Hyde & Seering, 1991) -- note ZVD above is exactly this
# construction applied to ZV convolved with itself (same mode twice).
######################################################################

def convolve_shapers(A1, T1, A2, T2):
    # Convolve two (A, T) impulse trains. Impulses landing at the same
    # time are merged (amplitudes added), so the combined shaper has at
    # most len(A1)*len(A2) impulses. Not normalized (consistent with the
    # single-mode get_*_shaper functions above; callers already divide by
    # sum(A) wherever a unit DC gain is required).
    combined = {}
    for a1, t1 in zip(A1, T1):
        for a2, t2 in zip(A2, T2):
            t = t1 + t2
            combined[t] = combined.get(t, 0.) + a1 * a2
    T = sorted(combined.keys())
    A = [combined[t] for t in T]
    return (A, T)

def get_multi_mode_shaper(base_init_func, shaper_freq, damping_ratio,
                          shaper_freq2, damping_ratio2):
    A1, T1 = base_init_func(shaper_freq, damping_ratio)
    A2, T2 = base_init_func(shaper_freq2, damping_ratio2)
    return convolve_shapers(A1, T1, A2, T2)

def get_zv2_shaper(shaper_freq, damping_ratio, shaper_freq2, damping_ratio2):
    return get_multi_mode_shaper(get_zv_shaper, shaper_freq, damping_ratio,
                                 shaper_freq2, damping_ratio2)

def get_mzv2_shaper(shaper_freq, damping_ratio, shaper_freq2, damping_ratio2):
    return get_multi_mode_shaper(get_mzv_shaper, shaper_freq, damping_ratio,
                                 shaper_freq2, damping_ratio2)

def get_zvd2_shaper(shaper_freq, damping_ratio, shaper_freq2, damping_ratio2):
    return get_multi_mode_shaper(get_zvd_shaper, shaper_freq, damping_ratio,
                                 shaper_freq2, damping_ratio2)

def get_ei2_shaper(shaper_freq, damping_ratio, shaper_freq2, damping_ratio2):
    return get_multi_mode_shaper(get_ei_shaper, shaper_freq, damping_ratio,
                                 shaper_freq2, damping_ratio2)

MultiModeShaperCfg = collections.namedtuple(
        'MultiModeShaperCfg',
        ('name', 'init_func', 'min_freq', 'max_damping_ratio'))

# Dual-mode shapers: each convolves two instances of the corresponding
# single-mode shaper above, one per resonant mode. Their init_func takes
# 4 arguments (shaper_freq, damping_ratio, shaper_freq2, damping_ratio2)
# instead of 2, so these are intentionally kept out of INPUT_SHAPERS /
# AUTOTUNE_SHAPERS -- shaper_calibrate.py's auto-tuning only searches a
# single frequency and everywhere calls init_func(freq, damping_ratio).
# See klippy/extras/input_shaper.py for how shaper_freq2_<axis> /
# damping_ratio2_<axis> are configured to use these.
MULTI_MODE_SHAPERS = [
    MultiModeShaperCfg(name='zv2', init_func=get_zv2_shaper,
                       min_freq=21., max_damping_ratio=0.99),
    MultiModeShaperCfg(name='mzv2', init_func=get_mzv2_shaper,
                       min_freq=23., max_damping_ratio=0.99),
    MultiModeShaperCfg(name='zvd2', init_func=get_zvd2_shaper,
                       min_freq=29., max_damping_ratio=0.99),
    MultiModeShaperCfg(name='ei2', init_func=get_ei2_shaper,
                       min_freq=29., max_damping_ratio=0.4),
]
