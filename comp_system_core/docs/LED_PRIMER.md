# LED Subsystem Primer

How `subsystems/led.py` decides what the strip shows, and how to add to it.

## 1. The strip

`wpilib.AddressableLED` treats the strip as a list of `LEDData`, one per pixel. You set each
pixel with `set_rgb(r, g, b)` or `set_hsv(h, s, v)` and send the whole list with
`led_strip.set_data(...)`. In 2027 there is no `start()`; the strip outputs as soon as it is
configured.

Three constants in `constants.LedConstants` describe the hardware:

| Constant | Now | Meaning |
| :--- | :--- | :--- |
| `k_led_count` | 16 | Pixels on the strip. Too small leaves the rest frozen on old colours. |
| `k_led_pwm_port` | 5 | SystemCore GPIO port (see the GPIO map at the top of `constants.py`). |
| `k_led_color_order` | `'RGB'` | Byte order of this strip. The wrong one swaps red and green. |

## 2. Indicators and modes

There are two layers, and **an indicator always wins over the mode**.

**Indicator (`Led.Indicator`)** - what is happening right now. This is what you almost
always see.

| Kind | Members | Looks like |
| :--- | :--- | :--- |
| Animated | `kRAINBOW`, `kCOOLBOW`, `kHOTBOW`, `kPOLKA` | A pattern that shifts along the strip. |
| Flashing | `kSUCCESS`, `kFAILURE`, `kWHITEFLASH`, `kINTAKEOFF` | On / off blink. |
| Flashing over the mode | `kSUCCESSFLASH`, `kFAILUREFLASH` | Blinks between its colour and the mode colour (`use_mode`). |
| Solid | `kINTAKEON` | `duty_cycle` 1, so it never turns off. |
| Pulse | `kREDPULSE`, `kBLUEPULSE` | The whole strip fades dim to bright (`'pulse': True`). |
| None | `kNONE` | No indicator: the mode shows through. |

**Mode (`Led.Mode`)** - the background, shown only when the indicator is `kNONE`.

| Member | Colour | When |
| :--- | :--- | :--- |
| `kNONE` | off | Normal. |
| `kDTAP` | orange | The Quest's pose stream stopped (`/QuestNav/quest_in_passthrough`). Set and cleared by `Led.periodic()` itself. |

How much of the strip the mode lights comes from `RobotState.State`'s `lit_leds`. Every
state the robot currently uses lights the whole strip, so in practice the mode is all on or
all off.

## 3. Who sets what

| Where | What it shows |
| :--- | :--- |
| `robot.py` `disabled_periodic()`, every 2 s | No DS or no alliance: `kPOLKA`. Alliance known: `kHOTBOW` (red) / `kCOOLBOW` (blue). Quest synced: `kREDPULSE` / `kBLUEPULSE`. So from the stands: rainbow = not synced yet, pulse = synced. |
| `DriveToPoseCustomControl` | `kPOLKA` while driving, then `kSUCCESSFLASH` or `kFAILUREFLASH` for 2 s. |
| `Intake_Set_RPM` (when given `led=`) | `kINTAKEON` while the rollers run; `kINTAKEOFF` for 1.5 s when they stop. |
| Button box 4 (brownout toggle) | `kFAILURE` (red) when brownout mode turns on, `kSUCCESS` (green) when it turns off. |
| Dashboard `LED Indicator` / `LED Mode` choosers | Whatever you pick, for testing. |

An indicator stays until something replaces it. Nothing clears the disabled pattern when
the robot is enabled, so it stays until the first command that sets an indicator.

## 4. The `periodic()` loop

Runs the LED update every 5th loop (10 Hz):

1. Read the DTAP flag and switch the mode to or from `kDTAP`.
2. If the indicator is not `kNONE`:
   - **pulse**: brightness from `pulse_brightness()`, a cosine between 10 % and 100 %.
   - **flashing**: toggle between `on_color` and the off colour when `1 / frequency`
     seconds times the duty cycle have passed.
   - **animated**: shift `animation_data` by one pixel every `1 / frequency` seconds.
3. Otherwise show the mode colour on `lit_leds` pixels from each end.
4. `led_strip.set_data(...)`.

Because the update is 10 Hz, a `frequency` above 5 cannot be shown faithfully.

## 5. Using it

Set an indicator and leave it:

```python
self.container.led.set_indicator(Led.Indicator.kPOLKA)
```

Show one for a while, then go back to `kNONE`. `set_indicator_with_timeout()` returns a
command, so it has to be scheduled:

```python
commands2.CommandScheduler.get_instance().schedule(
    self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))
```

It runs while disabled (it has to: the brownout toggle is used while disabled).

From a button or the dashboard, use the `SetLEDs` command (`commands/set_leds.py`).

## 6. Adding an indicator

Add one line to `Led.Indicator`. Copy the nearest existing one and change the name and
colours:

```python
kCLIMBING = {'name': "CLIMBING", "on_color": [255, 0, 255], "off_color": [0, 0, 0],
             "animated": False, "frequency": 2, "duty_cycle": 0.5, 'use_mode': False}
```

That is all. `indicators_dict` is built from the enum, so the new one appears in the
dashboard chooser by itself. Animated indicators also need `animation_data` (one colour per
pixel) and `use_hsv`.

`Led.set_leds()` takes either one colour or a list of colours, and an
optional `start_index` / `end_index`, so new patterns do not need their own pixel loops.

## 7. `RobotState` and the callback

`Led` registers `update_from_robot_state` with `RobotState`, which calls it whenever
`robot_state.state` is set. In 2025 that switched the mode between coral and algae. This
year the callback body is empty (`pass`): the hook is still wired, nothing uses it.
