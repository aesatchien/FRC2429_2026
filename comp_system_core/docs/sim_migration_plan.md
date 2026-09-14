# Simulation Migration Plan

Moving this project off pyfrc's `physics.py` and onto the WPILib-standard per-subsystem
simulation model.

Written 2026-09-13 against robotpy **2027.0.0a6.post1** (mamba env `robo2027_alpha`).

---

## 1. Why this document exists

### What broke

Through 2026, `robotpy sim` was pyfrc's command (`pyfrc.mains.cli_sim:PyFrcSim`). It built a
`PhysicsInterface`, loaded `<project>/physics.py`, instantiated our `PhysicsEngine`, and called
`update_sim()` every loop.

In 2027 `sim` and `test` moved **out of pyfrc and into wpilib core**
(`wpilib._impl.cli_sim:RobotSim`). That file loads `halsim_gui`, loads sim extensions, and calls
`robot_class.main()`. It has **no physics support of any kind** and never looks for `physics.py`.

Result: `physics.py` was never imported, our `Field2d` was never constructed, nothing published
`/SmartDashboard/Field`, and the sim GUI had no field to show.

### Installing pyfrc does NOT fix it

`pyfrc 2027.0.0a2` exists on PyPI but declares its entry points under the group `[robotpy]`.
robotpy-cli 2027 only scans `[robotpy_cli.2027]` (see `robotpy/main.py`, the
`for entry_point in entry_points(group="robotpy_cli.2027")` loop). Its `sim` command is silently
ignored. That release also still pins `robotpy-cli~=2024.0`. It is a placeholder, not a working
build. **Do not install it expecting the field back.**

### The stopgap that is in place now

`simulation/physics_interface.py` reimplements the three `PhysicsInterface` methods this project
actually uses (`get_pose`, `move_robot`, `drive`) and drives `PhysicsEngine.update_sim()` from
`MyRobot.simulationInit()` / `simulationPeriodic()`. It works, and it is a **bridge, not the
destination**. This document is the destination.

### Why bother migrating at all

`physics.py` was never WPILib. It is a robotpy-only framework from ~2015, built when WPILib had
no simulation of its own. Java and C++ teams never had a `physics.py`, a `PhysicsEngine`, or a
`physics_controller` - so there is nothing they migrated *to*. They were always doing it the
other way, using machinery we already have and have simply never used.

The concrete cost of staying put is one line, `simulation/swerve_sim.py:73`:

```python
# Update Robot Odometry (Perfect Odometry for Sim)
self.robot.container.swerve.pose_estimator.resetPosition(
    gyroAngle=pose.rotation(), wheelPositions=[SwerveModulePosition()] * 4, pose=pose)
```

Every loop, sim force-resets the pose estimator to ground truth. So in simulation odometry
cannot drift, cannot skid, and cannot disagree with vision - which means **the sim can never
reproduce an odometry or vision-fusion bug**, and those are most of the pose bugs we actually
get at competition.

---

## 2. The model we are moving to

Three pieces, all of which already exist in this robot:

1. **`simulationInit()` / `simulationPeriodic()` on the robot class.** Standard
   `IterativeRobotBase`. Verified on 2027.0.0a6.post1: `simulationInit()` fires once,
   `simulationPeriodic()` at 50 Hz immediately after `robotPeriodic()`. Neither runs on the real
   robot, so no `isSimulation()` guard is needed inside them.

2. **`Subsystem.simulationPeriodic()`.** `CommandScheduler.run()` already calls it on every
   registered subsystem whenever `RobotBase.isSimulation()`
   (`commands2/commandscheduler.py:241`). **All nine of our subsystems already inherit
   `commands2.Subsystem`**, so this hook is free everywhere - we have just never overridden it.

3. **Plant models that write back into the motor controllers' sim state.** `wpilib.simulation`
   provides `DCMotorSim`, `FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`,
   `DifferentialDrivetrainSim`. The vendors provide the other half: `rev.SparkSim` /
   `SparkMaxSim` / `SparkFlexSim` (plus `SparkRelativeEncoderSim` etc.), and phoenix6's
   `.sim_state` on each TalonFX.

The philosophical difference:

| | pyfrc `physics.py` | WPILib |
|---|---|---|
| Where sim lives | one god-object outside the robot | inside each subsystem |
| How it reaches the robot | `self.robot.container.<anything>` | writes to its own sensors |
| Ground truth pose | held by the simulator, robot cannot see it | none needed |
| Robot code path in sim | different (odometry is overwritten) | **identical to the real robot** |

`Field2d` is the tell. Java teams publish it from the **drive subsystem, off the pose estimator,
on the real robot too**. It is telemetry, not a simulation feature. Ours was owned by the
simulator, which is exactly why it vanished when a simulator detail changed.

---

## 3. Step 0 - `subsystems/motors.py` (do this first)

Not a subsystem, but the highest-leverage change. Everything else gets much smaller once it
exists.

`motors.py` already defines `DriveMotor` and `TurnMotor` Protocols with four implementations:
`RevDriveMotor`, `RevTurnMotor`, `TalonDriveMotor`, `TalonTurnMotor`.

**Work:**

- Add `sim_update(dt)` to both Protocols; implement it four times, mirroring how `describe()`
  is already done per vendor.
- Rev implementations drive `rev.SparkSim` / `SparkFlexSim`.
- Talon implementations drive `talon.sim_state` (`set_supply_voltage`,
  `set_raw_rotor_position`, `set_rotor_velocity`).
- Behind each, a `DCMotorSim` (or `FlywheelSim` for pure inertia) with the real gearing and
  moment of inertia.

**Bonus - this kills a live bug class.** `simulation/swerve_sim.py:49` reaches for
`SimDeviceSim(f'SPARK MAX [{can_id}]')` by CAN-ID string. That silently finds nothing the moment
a module changes vendor; its own comment already admits the drive sparks "would not exist
anyway" under the `comp_kraken` config. Going through `motors.py` removes the whole category.

---

## 4. Per subsystem

### Climber - *smallest, do it first to prove the pattern*

1 SparkMax, position controlled.

- `Climber.simulationPeriodic()`: an `ElevatorSim`, then `motor.sim_update(dt)`.
- `get_pos()` becomes honest (`robot.py` already converts it with `inchesToMeters`).

### Shooter - *easiest win, highest visual payoff*

3 SparkMax (hopper, indexer leader/follower) + 3 SparkFlex (flywheel leader/follower, roller).

- `Shooter.simulationPeriodic()`: one `FlywheelSim` per independent group.
- Flywheels are the simplest thing in WPILib to simulate.
- `current_rpm` / `shooter_on` / `current_hopper_rpm` feeding `blockhead_mech` become honest,
  including real spin-up time.

### Intake

2 SparkMax (roller leader/follower) + 1 SparkFlex (deploy) + a `DigitalInput` bumper switch.

- `SingleJointedArmSim` for the deploy arm, `FlywheelSim` for the rollers.
- Drive the bumper switch with `DIOSim(port).setValue(...)` off the gamepiece sim. **Nothing
  simulates that switch today.**
- Retires the hack at `robot.py:183`, which fakes the deploy angle by nudging
  `set_profile_setpoint()` by +/-1 per loop while disabled.

### Swerve + SwerveModule - *the big one*

Today `swerve_sim.py` integrates **commanded** module states into a ground-truth pose and
force-resets the estimator to it. Drive motors are never simulated. And the
`AnalogPotentiometer` - which is what actually closes the turn loop, and what **both**
`getState()` and `getPosition()` read - is never touched at all.

**`SwerveModule.simulationPeriodic(dt)`:**

- Drive: `self.drive_motor.sim_update(dt)`.
- Turn: integrate the commanded duty cycle through a `DCMotorSim` for the azimuth, then write
  the resulting angle **back through `AnalogInputSim(encoder_analog_port).setVoltage(...)`** so
  that `get_turn_encoder()` reads it.

> **MEASURED GOTCHA - the analog rail is 3.3 V, not 5 V.**
>
> With `AnalogPotentiometer(fullRange=2*pi, offset=-1)`, feeding 1.25 V reads 1.3800 rad. That is
> only consistent with a 3.3 V full scale (`(1.38 + 1) / 2pi = 0.3788`; `1.25 / 0.3788 = 3.3`).
> The TODO at `subsystems/swervemodule_2429.py:41` asks exactly this question - this is the
> answer, at least in simulation. Invert `k_analog_encoder_scale_factor` against **3.3 V** or
> every wheel will read about 1.5x its true angle.
>
> The inverse must also respect `k_reverse_analog_encoders` and each module's
> `turning_encoder_offset`.

**`Swerve.simulationPeriodic()`:**

- Pump each module, then drive `OnboardIMUSim` from the **actual** module states via
  `kinematics.toChassisVelocities(...)` - from `getState()`, **not**
  `get_desired_swerve_module_states()`.
- Keep the existing `setAngleX` + `setYaw` pair and its sign comment. That part of `swerve_sim`
  is correct and hard-won: `setAngleX` is the axis `Swerve` actually reads through
  `dc.k_imu_yaw_getter`, the two signals are independent, and OnboardIMU is CCW-positive where
  the old navX was CW-positive.
- **Delete the `resetPosition` cheat.** `Swerve.periodic()` already calls
  `pose_estimator.updateWithTime(...)` with `get_module_positions()`. Once the modules report
  simulated positions, odometry runs for real.
- Publish `Field2d` from `Swerve` off `get_pose()`, **unconditionally** (real robot too).

### Vision

`simulation/vision_sim.py` (229 lines) is **already the right shape** - it fakes the
`/Cameras/...` topics that `vision.py` subscribes to, which is exactly what this model wants. It
only needs an owner.

- Move it into `Vision.simulationPeriodic()`.
- Move the FOV cones and AprilTag poses onto Swerve's `Field2d`.
- The `isSimulation()` branches at `vision.py:103` and `vision.py:219` can probably collapse
  once `vision_sim` owns the fakery.

### Quest

Already self-contained behind `k_mock_questnav` and `isSimulation()`. Nothing to move;
optionally tidy the periodic bits into `simulationPeriodic()`.

### LED / Targeting / RobotState

No hardware to model. `AddressableLEDSim` exists if we ever want it. Nothing to do.

### BlockheadMech

Already driven from `robotPeriodic()` and already works on the real robot. **Leave it alone** -
it is the one piece already doing it the WPILib way.

---

## 5. Two things that have no subsystem

**Gamepiece sim** (`simulation/gamepiece_sim.py`, 46 lines) is field state, not a mechanism.
Put it on `RobotState` - it is already a `Subsystem` with a callback bus, and Intake needs to ask
it "am I on a piece?" in order to drive that DIO bumper switch.

**`_snap_to_live_tags()` and `_snap_to_quest()`** teleport ground truth from *real hardware* for
hardware-in-the-loop testing. These genuinely do not fit the per-subsystem model. Give them an
explicit `HardwareInTheLoop` helper pumped from `MyRobot.simulationPeriodic()` that resets the
swerve pose estimator directly. **Do not lose these in the migration** - they are the least
replaceable thing in the current sim.

---

## 6. The one real design decision

Does ground truth survive? Two honest options:

1. **No ground truth at all.** Encoders and IMU are simulated, odometry drifts naturally, vision
   corrects it. Most faithful.
2. **Keep it, but only as the source for the vision and gamepiece sims** - a camera should see
   where the robot *is*, not where it thinks it is - and publish it as a separate `Field2d`
   object so estimator error is visible on screen.

**Recommendation: (2).** It is what the `/Sim/ground_truth` topic already does, and it turns
estimator error from invisible into something you can watch drift in real time.

---

## 7. Order of work

| # | Step | Rough size |
|---|------|-----------|
| 1 | `motors.py` sim seam | an evening |
| 2 | Climber | an evening |
| 3 | Shooter | an evening |
| 4 | Intake (arm + DIO switch) | an evening |
| 5 | Swerve + SwerveModule; delete the odometry cheat | a weekend |
| 6 | Vision + gamepiece rehoming, HIL snap helper | an evening |
| 7 | Delete `physics.py`, `simulation/physics_interface.py`, `simulation/swerve_sim.py` | - |

**This is incremental, not a big bang.** `physics.py` keeps handling whatever has not been
migrated yet, as long as each piece is removed from `update_sim()` as its subsystem takes over.
The sim stays working at every step.

---

## 8. 2027 API changes hit along the way

Recorded here because they cost time and are not obvious:

| 2026 | 2027 |
|------|------|
| `ChassisSpeeds` | **`ChassisVelocities`** |
| `Pose2d.exp(twist)` | **`pose.transformBy(twist.exp())`** - `Twist2d.exp()` returns the `Transform2d` |
| `wpimath.geometry.*`, `wpimath.kinematics.*` | flattened to **`wpimath.*`** |
| `GenericHID.getPOV() -> int` (degrees) | **`-> POVDirection`** enum bitmask; does **not** compare equal to any int |
| `DriverStation.refreshData()` | **`DriverStationBackend.refreshData()`** |
| `robotInit()` | constructor |
| `testInit` / `testPeriodic` / `testExit` | `utilityInit` / `utilityPeriodic` / `utilityExit` |

Note on POV: `commands2` 2027.0.0a6 has **not** been ported. `povUp()`, `povDown()`, `povLeft()`,
`povRight()`, the diagonals **and** `povCenter()` all compare a `POVDirection` against an int
angle, so they are permanently false with no error. Use `hid.pov(POVDirection.UP)` instead.
