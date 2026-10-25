"""
Pin the swerve drive Kraken feedforward gains to the SysId fit (2026-10-25 hoots, four drive Krakens, R^2 0.997+).

kV was 0.810 (12 V / free speed, derived) and kS was 0.0 before the fit.  If someone puts the derived numbers back, or the
config stops using the constants, the robot quietly drives with a worse feedforward - nothing else would notice.
"""

import math

from subsystems.swerve_constants import ModuleConstants as M


def test_measured_gains_are_what_the_config_uses():
    slot = M.k_kraken_driving_config.slot0
    assert slot.k_v == M.k_kraken_kv == 0.76
    assert slot.k_s == M.k_kraken_ks == 0.18
    assert math.isclose(slot.k_a, M.k_kraken_ka)


def test_ka_is_entered_per_mps2_and_converted_to_wheel_rotations():
    # 0.06 V per (m/s^2) x 0.319 m per wheel rotation = 0.019 V per (wheel-rps / s), which is what the fit's per-motor 0.013-0.020 means
    assert math.isclose(M.k_kraken_ka, 0.06 * M.kWheelCircumferenceMeters)
    assert 0.013 <= M.k_kraken_ka <= 0.020


def test_derived_kv_is_kept_only_for_reference():
    assert math.isclose(M.k_kraken_kv_derived, 12.0 / M.k_kraken_wheel_free_speed_rps)
    assert abs(M.k_kraken_kv - M.k_kraken_kv_derived) / M.k_kraken_kv_derived < 0.10, "measured and derived kV should be close"
