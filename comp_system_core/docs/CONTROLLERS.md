# Robot Controls Reference (comp_system_core)

What every button does, read off `robotcontainer.py` on 2026-11-14. The bindings are the
truth; if this table and `bind_driver_buttons()` / `bind_bbox_buttons()` disagree, the code
wins and this file needs fixing. **Change a binding, change this table in the same commit.**

`comp_bot` has its own bindings (it differs: releasing a shoot button stops the intake
there, and it has a separate PS5 pad), so this table is for the SystemCore robot only.

## Driver pad

One pad on port 0. `constants.k_controller_type` picks the kind: `"PS5"` (the current
setting) or `"XBOX"`. The same `js.driver_*` names work for both, matched by position
(`helpers/joysticks.py` has the table), so there is one set of bindings.

| Xbox / PS5 | What it does | Code |
| :--- | :--- | :--- |
| **Left stick** | Drive (field-oriented). | `DriveByJoystickSubsystemTargeting` (default command) |
| **Right stick X** | Rotate. Ignored while tracking (RB held): targeting aims the robot. | same |
| **RT / R2** (analog) | Turbo. Speed is 44 % of max with the trigger out, rising to 100 % fully squeezed (rotation 50 % to 100 %). | `dc.kSlowModeCap`, `dc.kAngularSlowFloor` |
| **LB / L1** | **Two things.** Press: intake rollers on at 5000 RPM. While held: drive robot-oriented instead of field-oriented. | `Intake_Set_RPM(k_intake_teleop_rpm)`; read directly in the drive command |
| **RB / R1** (hold) | Track the hub: the robot turns to face it and the flywheel spins up to 3800 RPM. Release stops tracking; the flywheel keeps spinning. | `targeting.start_tracking()`, `set_shooter_rpm(k_fire_up_speed)` |
| **A / Cross** (hold) | Shoot at the RPM targeting computes from distance. Intake rollers drop to 500 RPM, the shooter spins up and feeds; the intake lifts to 75 degrees after 1 s and to 105 degrees after 2 s to push balls in. Release: shooter stops, intake goes down, rollers back to 5000 RPM. | `ShootingCommand` + `Intake_Deploy('shoot' / 'shoot2')` |
| **X / Square** (hold) | The same cycle at a fixed 3300 RPM. | `ShootingCommand(rpm=3300)` |
| **B / Circle** (hold) | Feed (lob) at 5800 RPM with the indexer at 5600 RPM. Same intake sequence and release as A. | `ShootingFeedingCommand(rpm=k_shooter_max_speed)` |
| **Y / Triangle** | **Resets the whole pose, not just the heading**: the estimate jumps to 3.2 m out from our alliance wall on the field centreline, facing away from the wall, and the Quest is unsynced. Hold 0.5 s to also sync the Quest to the pose estimate. Works while disabled. | `ResetFieldCentric`, `quest_sync_odometry()` |
| **LT / L2** (hold) | X-lock the wheels. | `SwerveSetX` |
| **D-pad up** | Intake up (148 degrees). | `Intake_Deploy('up')` |
| **D-pad down** | Intake down, rollers on at 5000 RPM. | `Intake_Deploy('down')`, `Intake_Set_RPM` |
| **D-pad left / right** | Simulation only: hold to drive onto the game piece the left / front camera sees. | `DriveToPoseCustomControl` |
| **View / Create** (left small button) | Intake rollers off. | `Intake_Set_RPM(0)` |
| **Menu / Options** (right small button) | Eject: intake down, rollers at -2500 RPM, hopper at -5000 RPM. **It is a press, not a hold** - nothing stops on release. Stop it with View/Create and by shooting or `HopperOff`. | `Intake_Deploy('down')`, `Intake_Set_RPM(-k_intake_default_rpm)`, `set_hopper_rpm` |

## Button box (port 2)

Buttons are numbered 1 to 12 as on the box. In code they are `js.bbox_1_1` .. `js.bbox_1_12`.
(a7 numbers raw buttons from 0, so `bbox_1_3` is `button(2)`; only `helpers/joysticks.py`
should ever contain that number.)

| Button | What it does | Code |
| :--- | :--- | :--- |
| **1** | Tell the intake it is at the bottom (encoder := 0 degrees). Works disabled. | `intake.zero_intake()` |
| **2** | Tell the intake it is at the top (encoder := 148 degrees). Works disabled. | `intake.set_angle_max()` |
| **3** (hold) | Intake up with rollers and hopper off; and after 0.2 s **`SwerveTest` takes the drivetrain away from the driver** and runs one module at a time at 0.25 m/s until release. See the note below. | `Intake_Deploy('up')`, `SwerveTest` |
| **4** | Toggle brownout mode: drive supply current limit 50 A <-> 40 A. LEDs flash red when it turns on, green when it turns off. Works disabled. **Also, while it is held, afterburner**: full speed whatever the trigger says. See the note below. | `swerve.set_brownout_mode()`; raw read in the drive command |
| **5** | Stop using the Quest for odometry. | `quest_enabled_toggle('off')` |
| **6** | Use the Quest for odometry again. | `quest_enabled_toggle('on')` |
| **7** | Unsync the Quest (its pose is ignored until the next sync). | `quest_unsync_odometry()` |
| **8** | Intake up, rollers off. | `Intake_Deploy('up')`, `Intake_Set_RPM(0)` |
| **9** | Intake to the shooting angle (75 degrees), rollers at 500 RPM. | `set_intake_position`, `set_intake_rpm` |
| **10** | Intake down, rollers at 3000 RPM. | `Intake_Deploy('down')`, `Intake_Set_RPM(3000)` |
| **11** (switch) | Shooter RPM offset -180 while on, back to 0 when off. | `shooter.set_shooting_offset()` |
| **12** (switch) | Shooter RPM offset +180 while on, back to 0 when off. | same |

Nobody syncs the Quest from the button box on purpose: a sync should come from a good
AprilTag fix (the automatic sync while disabled, or the driver's long Y press), not from
an operator's guess.

**Buttons 3 and 4 need a decision.** Two things are wrong and they are tangled:

- The afterburner is read as raw button index 3. In `comp_bot` (buttons numbered from 1)
  that is button 3. In a7 (numbered from 0) it is button 4, the brownout toggle. The index
  was not shifted in the port, so on this robot pressing the brownout toggle also gives
  full speed for as long as it is held.
- In both projects button 3 runs `SwerveTest` after 0.2 s, which takes the drivetrain from
  the driver. So even where the afterburner is on button 3 (`comp_bot`), it lasts 0.2 s.

Decide which button the afterburner belongs on and whether `SwerveTest` should be on a
match button at all, then fix `get_raw_button(3)` in
`commands/drive_by_joystick_subsystem_targeting.py` (use a `js.bbox_*` name, not a number).

## Co-driver Xbox pad (port 1)

Declared in `helpers/joysticks.py` (`js.copilot_*`), nothing is bound to it.

## Dashboard buttons

`initialize_dashboard()` publishes commands under `/SmartDashboard/Command/`: intake
stow / deploy / shoot / on / off / reverse / brake / idle / calibrate, hopper, indexer and
shooter on / off / reverse, LED choosers, `CANStatus`, the Quest commands (from
`subsystems/quest.py`), and in simulation `SimKickRobot` / `SimKickQuest`.

## Words used above

- **Hold** (`while_true`): starts on press, is cancelled on release. A command that finishes
  by itself (an `Intake_Deploy`, an `InstantCommand`) is not undone on release - that is why
  Menu/Options is a press.
- **Press** (`on_true`): fires once.
- **Works disabled**: the command has `.ignoring_disable(True)`.
