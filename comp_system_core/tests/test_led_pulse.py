"""
Quest-synced LED pulse: while disabled, once the Quest has synced, the strip pulses slowly in
the alliance color (robot.py disabled_periodic) so the sync is visible from the bleachers.
"""

import math

from wpilib import AddressableLED

from subsystems.led import Led, led_color_order, pulse_brightness


def test_pulse_fades_between_floor_and_full():
    f = 0.5   # 2 s per cycle
    samples = [pulse_brightness(t / 100, f) for t in range(0, 201)]
    assert abs(min(samples) - 0.1) < 1e-3 and abs(max(samples) - 1.0) < 1e-3
    assert abs(pulse_brightness(0.0, f) - 0.1) < 1e-9      # dim at the start of a cycle
    assert abs(pulse_brightness(1.0, f) - 1.0) < 1e-9      # full at the half cycle
    steps = [abs(b - a) for a, b in zip(samples, samples[1:])]
    assert max(steps) < 0.05                                # smooth - no on/off jump


def test_blue_pulse_lights_only_blue_and_changes_brightness(container, monkeypatch):
    led = container.led
    led.set_indicator(Led.Indicator.kBLUEPULSE)
    seen = []
    import subsystems.led as led_module
    for t in (0.0, 0.5, 1.0):
        monkeypatch.setattr(led_module.Timer, 'get_timestamp', staticmethod(lambda t=t: t))
        led.counter = 4            # periodic() renders on every 5th call
        led.periodic()
        rgb = [(d.r, d.g, d.b) for d in led.led_data]
        assert all(r == 0 and g == 0 for r, g, b in rgb), "blue pulse must not light red/green"
        seen.append(rgb[0][2])
    assert seen[0] < seen[1] < seen[2], f"brightness should rise over the half cycle, got {seen}"
    led.set_indicator(Led.Indicator.kNONE)


def test_indicator_names_are_unique():
    # indicators_dict is keyed by name - a duplicate silently hides one from the dashboard
    names = [i.value['name'] for i in Led.Indicator]
    assert len(names) == len(set(names)), names


def test_led_color_order_falls_back_to_grb_on_a_bad_name(capsys):
    assert led_color_order('RGB') == AddressableLED.ColorOrder.RGB
    assert led_color_order(' rgb ') == AddressableLED.ColorOrder.RGB       # case/space forgiven
    for bad in ('RGBW', 'xyz', '', None, 3):
        assert led_color_order(bad) == AddressableLED.ColorOrder.GRB        # never raises
    assert 'using GRB' in capsys.readouterr().out
