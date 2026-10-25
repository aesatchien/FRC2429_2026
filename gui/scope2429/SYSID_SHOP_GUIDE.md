# Shop test: confirm or redo the swerve drive feedforward (kS / kV / kA)

Scope2429 estimates the drive Krakens' feedforward from match hoots (report section "Drive motor feedforward (hoot)",
or `python sysid.py <logs folder>` for all matches pooled). That is an estimate made from closed-loop driving. This
procedure is the check: the same three numbers from a controlled test.

What the match data said (14 CAEL matches, 2026): kV 0.78 to 0.80 V per wheel rps (about 2.5 V per m/s), within about 2%
between matches. kS 0.1 to 0.35 V, scattered; kA about 0.02 V per rps/s (about 0.06 V per m/s²), stable but probably biased.
The match hoot logs MotorVoltage only every 250 ms, which is the main weakness.

## 0. What the shop test should settle

| Number | Match estimate | What the shop test tells you |
|---|---|---|
| kV | good | confirms it (expect within 5%) |
| kS | rough | the real value: the voltage at which the wheel just starts to turn |
| kA | biased | the real value: from the voltage step |

Pass criterion: shop kV within 5% of the match kV, shop kA within 30% of the match kA. If not, trust the shop number
and look for the reason (wheel tread, carpet, gear ratio entered wrong).

## 1. Make the hoot useful (do this first, it also improves every future match log)

**Done in `comp_system_core`** (SystemCore robot): `TalonDriveMotor.__init__` in `subsystems/motors.py` now sets MotorVoltage,
StatorCurrent, SupplyCurrent and Acceleration to `constants.k_drive_hoot_rate_hz` (100 Hz; position and velocity were
already 100 Hz), and `swerve.py` already runs `SignalLogger.start()`. Set `k_drive_hoot_rate_hz = 4` to go back to the quiet
default. `comp_bot` (roboRIO) does not have this yet. The snippet below is what that change amounts to, for any other robot.

The default status rate is the problem. In the robot code, before logging starts, for each drive TalonFX:

```python
from phoenix6 import BaseStatusSignal, SignalLogger

# once per drive TalonFX (self.drive_motor), at start-up, before logging starts
BaseStatusSignal.set_update_frequency_for_all(
    100,                                  # 250 if the drive motors are on a CANivore
    self.drive_motor.get_motor_voltage(), self.drive_motor.get_velocity(), self.drive_motor.get_position(),
    self.drive_motor.get_stator_current(), self.drive_motor.get_supply_current(), self.drive_motor.get_acceleration())
self.drive_motor.optimize_bus_utilization()      # every other signal stays slow
SignalLogger.start()                             # hoot logging (leave it on in matches too)
```
Check afterwards in Scope2429 or `python sysid.py` that MotorVoltage in the converted hoot is about 100 Hz, not 4 Hz.
Leave this on for matches: the 4 drive motors at 100 Hz is a small CAN load.

## 2. Setup (about 15 minutes)

- Clear floor, at least 4 m straight run, the same carpet as the field if you have it. A second person holds the e-stop.
- Fresh battery, 12.5 V or more at rest. Charge between tests if it drops below 12 V. (The fit drops samples under 9 V.)
- All four modules pointed straight ahead, wheels zeroed. Turn off anything that fights the test: heading hold, vision
  correction, drive-pose feedback.
- Drive at the real configuration: the gear ratio, wheel diameter and current limits you run in matches. Do not change
  them for the test. Keep the stator current limit (if you use one) from clipping the ramp: the test voltage stays at or
  below 6 V per motor for the quasistatic test.
- Test on the carpet with the robot's weight on the wheels. (Optional second run on blocks, wheels off the floor, gives
  the motor and gearbox alone. The difference between the two runs is the carpet and rolling friction.)

## 3. The tests (WPILib SysId routine, all four modules driven together)

**Done in `comp_system_core`** (`subsystems/swerve_sysid.py`). It is off by default. For a shop session set
`k_enable_sysid = True` in `constants.py` (ramp, step and timeout are `k_sysid_ramp_rate` / `k_sysid_step_voltage` /
`k_sysid_timeout`), deploy, and four commands appear on the dashboard under `Command/SysId/` (`1 QuasistaticForward`,
`2 QuasistaticReverse`, `3 DynamicForward`, `4 DynamicReverse`). Enable the robot, then start one from the driver GUI,
AdvantageScope or Elastic. They are dashboard buttons, not joystick or keyboard bindings, so no controller button is used.
The commands only run while the robot is enabled, and they take over the swerve from the joystick drive until they end.
Set it back to `False` before matches. The sketch below is what that code does.

Use RobotPy's `SysIdRoutine` (`from commands2.sysid import SysIdRoutine`) with the Phoenix 6 `SignalLogger`, so the data
lands in a hoot, not in NetworkTables. Each module's drive motor gets the same raw voltage, with all four turn motors
holding 0 degrees. Sketch (check the argument names against the robotpy-commands-v2 version you have installed):

```python
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from phoenix6.controls import VoltageOut

volts = VoltageOut(0).with_enable_foc(False)       # plain voltage, same as the match feedforward
self.sysid = SysIdRoutine(
    SysIdRoutine.Config(rampRate=1.0, stepVoltage=4.0, timeout=6.0,
                        recordState=lambda state: SignalLogger.write_string("state", SysIdRoutine.stateEnumToString(state))),
    SysIdRoutine.Mechanism(lambda v: [m.set_control(volts.with_output(v)) for m in self.drive_motors],
                           lambda log: None,       # the hoot already has voltage / position / velocity
                           self, "swerve-drive"))
# bind to buttons while disabled-in-shop only:  self.sysid.quasistatic(SysIdRoutine.Direction.kForward)  /  .dynamic(...)
```

1. **Quasistatic forward**: voltage ramp of 1 V/s from 0 until about 2.5 to 3 m of travel or 6 V. The robot accelerates
   slowly, so kA barely matters: this gives kS (the intercept) and kV (the slope).
2. **Quasistatic reverse**: the same backwards.
3. **Dynamic forward**: a step of 4 V (the default 7 V is too much for a short lane; use less if you run out of floor).
   This excites the acceleration: it gives kA.
4. **Dynamic reverse**: the same backwards.

Run in this order, forward then reverse each time, and drive back to the start between tests. The `state` string written
by `recordState` is what the SysId analyzer uses to split the tests. Repeat each of the four tests 2 to 3 times.

Safety: a step to 4 V on four swerve modules is a lively robot. Robot on the floor only with a clear lane, e-stop in
hand, nobody in front of it, bumpers on. Never on the cart or on blocks with the wheels touching anything.

## 4. Analysis

Option A (recommended, an independent check): convert the hoot to wpilog (`owlet file.hoot out.wpilog -f wpilog`, or let
Scope2429 do it) and open it in the WPILib SysId analyzer (Analyzer tab, load the data, choose "Drive" as the mechanism
type, units rotations). It gives kS, kV, kA with their own diagnostics (r², the fit plot, the "residuals" plot).

Option B (same code as the match estimate): run
`python sysid.py <folder with the hoot + robot log>` on the test day's logs, restricted to the test period. Same model,
same units, so the shop numbers compare directly with the match numbers.

Convert units carefully. The hoot logs the wheel in motor rps with the gear ratio applied; the wheel travels
about 0.315 m per rps on this robot (4 inch wheels). So `kV [V/(m/s)] = kV [V/rps] / 0.315`.

## 5. How to read the result

- The quasistatic plot (volts vs. steady speed) should be a straight line. Its intercept is kS, its slope is kV. A kink
  near 0 means static friction is higher in one direction (a stiff module): compare forward and reverse.
- All four modules should agree on kV within about 5%. A module that is off by more is dragging, has a different tread,
  or has a wrong gear ratio entered.
- kA from the dynamic test is the number the match fit is least sure of. If the shop kA is far from the match kA, use the
  shop one.
- r² should be above 0.95 for each test. If not: slip (lower the step), a brownout (check battery), or a module not at 0 degrees.

## 6. What to do with the numbers

Put kS, kV, kA (per module or the average) in the drive velocity feedforward, then tune kP by looking at the
commanded-vs-actual velocity in Scope2429's motor rpm indicators: the error should stay inside the deviation threshold you
set in the main GUI during a hard acceleration. Re-run the match estimate on the next event's logs to see that it moved
the way the test said it would.
