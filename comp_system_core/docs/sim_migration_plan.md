# Simulation Migration — pyfrc `physics.py` → the WPILib/Java model

**Status: DONE on 2026-09-14, on branch `migrate-2027a7`, against robotpy 2027.0.0a7
(mamba env `robo2027_a7`).** `physics.py`, `simulation/physics_interface.py` and
`simulation/swerve_sim.py` are gone. Sections 0–5 describe what was built and where it lives;
section 6 is what is still open; section 7 is what the migration turned up that was not
about simulation at all.

---

## 0. What changed, in one table

| | pyfrc `physics.py` (before) | WPILib model (now) |
|---|---|---|
| Where sim lives | one god-object outside the robot | `simulation_periodic()` on each subsystem |
| How it reaches the robot | `self.robot.container.<anything>` | writes into its own controllers' sim state and its own sensors |
| Ground truth pose | held by the simulator, robot could not see it | integrated by `Swerve`, published on `/SmartDashboard/Sim/ground_truth`, drawn as `GroundTruth` |
| Odometry in sim | **force-reset to ground truth every loop** | runs for real; can drift, skid and disagree with vision |
| Robot code path in sim | different (`is_real()` guard around odometry) | **identical to the real robot** |
| Field2d | created by `physics.py`, sim only | owned by `Swerve`, published on the real robot too |

Why it mattered: the old `reset_position(...)` cheat meant the sim could never reproduce an
odometry or vision-fusion bug, which are most of the pose bugs we actually get.

## 1. The seam: `subsystems/motors.py`

`DriveMotor` and `TurnMotor` gained `sim_update(dt, vbus) -> amps` (and `TurnMotor` gained
`sim_azimuth_rad()`), implemented four times like everything else in that file:

- `_RevPlant`: a `DCMotorSim` behind `rev.SparkFlexSim` / `SparkMaxSim`. The plant's motor
  RPM is handed to `SparkSim.iterate()`, which closes the Spark's own velocity loop (kP, kV,
  MAXMotion) and moves its encoder, so the encoder reads motor rotations exactly as the real
  one does and `motors.py`'s unit factors apply unchanged.
- `_TalonPlant`: a `DCMotorSim` behind `TalonFXSimState`. Reads `motor_voltage`, writes rotor
  position/velocity; `sensor_to_mechanism_ratio` in the device config divides them back down.
  Works through `helpers/phoenix6_compat.py` like everything else Phoenix.

Plants are built lazily on the first `sim_update()`, so nothing runs on the real robot.
Inertias and masses live in `constants.SimConstants` and are rough by design — they set
spin-up times and current draw, and no gain is derived from them.

**REV sim gotcha (measured on robotpy-rev 2027.0.0a7.post1):** `SparkSim.iterate()`
overwrites the encoder with the sim's own integrated position, and neither
`RelativeEncoder.set_position()` nor `SparkRelativeEncoderSim.set_position()` moves that
internal position — only `SparkSim.set_position()` does. So any `set_position()` the robot
code makes is silently undone next loop unless it is mirrored into the `SparkSim`.
`_RevPlant.seed()` does that for the swerve adapters, and `Intake._set_deploy_angle_deg()`
does it for the deploy arm. Anything new that re-zeroes a Spark encoder in sim needs the same.

## 2. Per subsystem — where each piece went

| Subsystem | `simulation_periodic()` does |
|---|---|
| **SwerveModule** | `sim_update()` both motors; write the turn plant's azimuth back through `AnalogInputSim` so `get_turn_encoder()` — and therefore both `get_position()` and `getState()` — read it. The analog rail is **3.3 V** in sim (re-verified: 3.3 V reads exactly one turn), and the inverse respects `k_reverse_analog_encoders` and each module's offset. |
| **Swerve** | pump the modules; `OnboardIMUSim.set_angle_x/set_yaw/set_gyro_rate_z` from the chassis velocities the **measured** module states imply (CCW-positive — the sign flip the navX needed is gone); integrate ground truth as a pose exponential; `BatterySim` from the summed amps. The `is_real()` guard around `pose_estimator.update_with_time()` is deleted. Also owns and publishes the Field2d, on the real robot too. |
| **Shooter** | one `FlywheelSim` per group (flywheel ×2 Vortex, roller, indexer ×2 NEO, hopper) behind the leader Spark's sim; followers get the leader's speed so their encoders read. `is_at_speed()` is now honest, spin-up included. |
| **Intake** | `SingleJointedArmSim` for the deploy (gravity with 0° = horizontal, the same assumption the `ArmFeedforward` makes) and a `FlywheelSim` for the rollers. Drives the bumper switch through `DIOSim` from the simulated arm angle when the switch is enabled — it is an at-the-bottom switch, which is what auto-calibration uses it for, not a game-piece sensor as the old plan guessed. |
| **Climber** | `ElevatorSim` behind a `SparkMaxSim`. Pattern only — the subsystem is still half written and not constructed by `RobotContainer`. |
| **Vision** | owns `simulation/vision_sim.py`, fed ground truth from NT. Reads the game pieces back off the field's `Gamepieces` object, so it depends on nothing but the shared Field2d. |
| **RobotState** | owns `simulation/gamepiece_sim.py` (field state, not a mechanism), fed ground truth from NT. |
| **Quest** | untouched; already self-contained behind `k_mock_questnav`. |
| **LED / Targeting** | nothing to model. |

`robot.py`'s `simulation_periodic()` pumps the two things that are not mechanisms:
`simulation/hil_snap.py` (`HardwareInTheLoop`: snap **ground truth** to a real camera's tag or a
real Quest — the estimator is deliberately not touched, it learns from the vision measurements
like on the robot) and `simulation/ghost_robot.py` (draws the auto goal pose and shot line).
`BlockheadMech` was left alone, except that the intake ligament now draws the **measured** arm
angle; the disabled-mode hack that nudged the profile setpoint a degree a loop is gone.

## 3. Field2d on a7

a7 cannot publish a `Field2d` natively (see `_NATIVE_GAP` in `helpers/dashboard.py`), and it
cannot enumerate a field's objects either. So `dashboard.field("Field")` hands out the one
shared field and `dashboard.field_object("Gamepieces")` hands out objects **and registers
them** so the hand publisher includes them each loop. An object fetched with a bare
`field.get_object()` is invisible on the dashboard — use `field_object()`.

## 4. Design decision: ground truth survives

Option 2 from the original plan. Ground truth exists only as the source for the camera and
gamepiece sims and as a second robot on the field, so estimator error is something you can
watch. Nothing in the robot reads it for control (`Swerve.sim_get_ground_truth()` is prefixed
`sim_` for that reason).

## 5. How it is verified

`tests/test_simulation.py` runs the whole robot through WPILib's test harness with a simulated
Xbox controller and asserts on what the **robot reads back**: drive encoders and odometry move
under full stick and ground truth agrees; the IMU heading turns CCW under right stick and the
estimator follows; `set_x()` shows up on all four absolute encoders; the flywheel spins up
visibly and `is_at_speed()` goes True; the deploy arm reaches its setpoint in degrees; a
teleported ground truth consumes a game piece. It is one test on purpose — REV's sim registry
survives the harness's robot teardown, so a second robot in one process fails on duplicate
CAN ids. `robotpy test` → 13/13.

The headless run (`robotpy sim --nogui`) is clean: zero `Traceback` lines. (Killing it with
`timeout` prints a faulthandler dump that looks like an access violation in
`mechanism_publisher`; that is the kill, not a crash — the same dump appears on the
pre-migration commit and never appears while the program is running.)

## 6. Still open

- **`vision.py` sim branches.** `get_strafe()` still fakes "we are on tag 18" under
  `is_simulation()`, and `periodic()` publishes the FPGA time as match time in sim. Both
  predate the vision sim and are independent of it; with `k_disable_vision_sim = True` by
  default the camera sim publishes no targets, so the `get_strafe()` hack is the only thing
  keeping strafe-based commands alive in sim. Decide whether to flip the default and delete it.
- **Plant numbers are guesses.** `SimConstants.k_*_moi` etc. Tune when something looks
  wrong on screen; nothing on the robot depends on them.
- **Intake gravity model.** 0° = horizontal, so the stowed arm (148°) leans past vertical and
  settles on the 153° hard stop when unpowered. That is what the code's own feedforward
  assumes; if the real zero is not horizontal, fix the `ArmFeedforward` offset and this
  together.
- **Delete `helpers/mechanism_publisher.py` and the `_NATIVE_GAP` section** once `log_to()`
  stops raising on a newer alpha — unchanged from the a7 migration notes.

## 7. Bugs the migration turned up that were not about simulation

All fixed on this branch; all would have failed on the **real robot**:

1. **`Intake` read the deploy encoder as degrees.** a7's rev deleted
   `positionConversionFactor`, and `constants.py` said intake.py applied
   `k_deploy_position_factor` instead — it never did. One motor rotation looked like one
   degree, so the arm would have moved ~6.6× too far per degree of setpoint. Now funnelled
   through `get_angle_deg()` / `_set_deploy_angle_deg()`.
2. **`Command*Controller.get_hid()` is the `CommandGenericHID` wrapper on a7**, which has
   none of the named getters. `robot.py`'s `report_gamepads()` (`k_debug_gamepads` is True)
   would have raised in `disabled_periodic` the moment a pad was plugged in, and the default
   drive command would have raised on the first teleop loop. Both invisible with no DS
   attached, because the connected check skips the line. Now `get_controller()`.
   `drive_by_joystick_swerve.py` had the same plus the removed `_axis` suffix.
3. **`Translation2d.angle()` returns `None` for the zero vector on a7.** `sim_utils` did
   arithmetic on it; standing exactly on a game piece was a `TypeError`.
