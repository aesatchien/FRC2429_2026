# Simulation Migration Plan — pyfrc `physics.py` → the WPILib/Java model

Moving simulation off pyfrc's `physics.py` and onto per-subsystem `simulation_periodic()`.

Rewritten 2026-09-14 against **robotpy 2027.0.0a7** (`C:/FRC/2026/venv_a7`). Every API name
below was checked against that environment — a7 renamed the entire Python surface, so an
older copy of this plan would send you looking for methods that no longer exist.

---

## 0. Where we are right now

`robotpy sim` moved out of pyfrc and into wpilib core in 2027 (`wpilib._impl.cli_sim`), and
the core version has **no physics support at all** — it never loads `physics.py`. We bridged
that with `simulation/physics_interface.py`, which reimplements the three `PhysicsInterface`
methods this project uses and pumps `PhysicsEngine.update_sim()` from
`MyRobot.simulation_init()` / `simulation_periodic()`.

**That bridge works and is not the destination.** This document is the destination.

Do not reinstall pyfrc to "fix" this. pyfrc 2027.0.0a2 registers its commands under the old
`[robotpy]` entry-point group, which robotpy-cli 2027 does not scan, so installing it does
literally nothing.

## 1. Why bother

`physics.py` was never WPILib. It is a robotpy-only framework from ~2015, built when WPILib
had no simulation of its own. Java and C++ teams never had a `physics.py`, a `PhysicsEngine`
or a `physics_controller` — so there is nothing they migrated *to*. They were always doing it
the other way, using machinery this robot already has and has never used.

The cost of staying put is one line, `simulation/swerve_sim.py:73`:

```python
# Update Robot Odometry (Perfect Odometry for Sim)
self.robot.container.swerve.pose_estimator.reset_position(
    gyro_angle=pose.rotation(), wheel_positions=[SwerveModulePosition()] * 4, pose=pose)
```

Every loop, sim force-resets the pose estimator to ground truth. So in simulation odometry
cannot drift, cannot skid and cannot disagree with vision — which means **the sim can never
reproduce an odometry or vision-fusion bug**, and those are most of the pose bugs we actually
get at competition.

## 2. The model we are moving to

Three pieces, all of which already exist in this robot and are verified present on a7:

1. **`simulation_init()` / `simulation_periodic()` on the robot class.** Standard
   `IterativeRobotBase`. `simulation_init()` fires once, `simulation_periodic()` at 50 Hz
   immediately after `robot_periodic()`. Neither runs on the real robot, so nothing inside
   them needs an `is_simulation()` guard.

2. **`Subsystem.simulation_periodic()`.** `CommandScheduler.run()` already calls it on every
   registered subsystem under `if RobotBase.is_simulation():` — confirmed in the a7 source.
   **All nine of our subsystems already inherit `commands2.Subsystem`**, so the hook is free
   everywhere; we have simply never overridden it.

3. **Plant models that write back into the controllers' sim state.** `wpilib.simulation`
   provides `DCMotorSim`, `FlywheelSim`, `ElevatorSim`, `SingleJointedArmSim`,
   `DifferentialDrivetrainSim`. The vendors provide the other half:
   `rev.SparkSim(spark, motor)` with `.iterate(velocity, vbus, dt)` and
   `.get_relative_encoder_sim()`, and phoenix6's `.sim_state` on each TalonFX.

| | pyfrc `physics.py` | WPILib |
|---|---|---|
| Where sim lives | one god-object outside the robot | inside each subsystem |
| How it reaches the robot | `self.robot.container.<anything>` | writes to its own sensors |
| Ground truth pose | held by the simulator, robot cannot see it | none needed |
| Robot code path in sim | different (odometry is overwritten) | **identical to the real robot** |

## 3. Step 0 — `subsystems/motors.py` (do this first)

Not a subsystem, but the highest-leverage change; everything else shrinks once it exists.

`motors.py` already defines `DriveMotor` and `TurnMotor` Protocols with four implementations
(`RevDriveMotor`, `RevTurnMotor`, `TalonDriveMotor`, `TalonTurnMotor`), and since the a7
migration it already owns unit conversion for both vendors. Simulation belongs in exactly the
same seam.

- Add `sim_update(dt)` to both Protocols; implement four times, mirroring how `describe()` is
  already done per vendor.
- Rev side: `rev.SparkSim` / `SparkFlexSim`, `.iterate(velocity, vbus, dt)`.
- Talon side: `talon.sim_state` (`set_supply_voltage`, `set_raw_rotor_position`,
  `set_rotor_velocity`).
- Behind each, a `DCMotorSim` or `FlywheelSim` with the real gearing and inertia.

**This also kills a live bug class.** `simulation/swerve_sim.py:48` reaches for
`SimDeviceSim(f'SPARK MAX [{can_id}]')` by CAN-ID string. That silently finds nothing the
moment a module changes vendor; its own comment already admits the drive sparks "would not
exist anyway" under `comp_kraken`.

## 4. Per subsystem

### Climber — smallest, do it first to prove the pattern
1 SparkMax, position controlled.
- `Climber.simulation_periodic()`: an `ElevatorSim`, then `motor.sim_update(dt)`.
- `get_pos()` becomes honest.

### Shooter — easiest win, highest visual payoff
3 SparkMax + 3 SparkFlex.
- One `FlywheelSim` per independent group (flywheel, hopper, indexer, roller).
- Flywheels are the simplest thing in WPILib to simulate.
- `current_rpm` / `shooter_on` feeding `blockhead_mech` become honest, spin-up time included.

### Intake
2 SparkMax + 1 SparkFlex + a `DigitalInput` bumper switch.
- `SingleJointedArmSim` for the deploy arm, `FlywheelSim` for the rollers.
- Drive the switch with `DIOSim(port).set_value(...)` off the gamepiece sim. **Nothing
  simulates that switch today.**
- Retires the hack at `robot.py:248`, which fakes the deploy angle by nudging
  `set_profile_setpoint()` by ±1 per loop while disabled.

### Swerve + SwerveModule — the big one

Today `swerve_sim.py` integrates **commanded** module states into a ground-truth pose and
force-resets the estimator to it. Drive motors are never simulated. And the
`AnalogPotentiometer` — which is what actually closes the turn loop, and what **both**
`get_state()` and `get_position()` read — is never touched at all.

**`SwerveModule.simulation_periodic(dt)`:**
- Drive: `self.drive_motor.sim_update(dt)`.
- Turn: integrate the commanded duty cycle through a `DCMotorSim` for the azimuth, then write
  the resulting angle **back through `AnalogInputSim(encoder_analog_port).set_voltage(...)`**
  so `get_turn_encoder()` reads it.

> **MEASURED GOTCHA — the analog rail is 3.3 V, not 5 V.** Re-verified on a7: with
> `AnalogPotentiometer(3, 2*pi, -1.0)`, feeding 1.25 V reads 1.3800 rad, which back-solves to
> exactly 3.30 V full scale. The TODO at `subsystems/swervemodule_2429.py:43` asks this
> question — that is the answer, at least in simulation. Invert
> `k_analog_encoder_scale_factor` against **3.3 V** or every wheel reads ~1.5× its true angle.
> The inverse must also respect `k_reverse_analog_encoders` and each module's
> `turning_encoder_offset`.

**`Swerve.simulation_periodic()`:**
- Pump the modules, then drive `OnboardIMUSim` from the **actual** module states via
  `kinematics.to_chassis_velocities(...)` — from `get_state()`, **not**
  `get_desired_swerve_module_states()`.
- Keep the existing `set_angle_x` + `set_yaw` pair and its sign comment. That part of
  `swerve_sim` is correct and hard-won: `set_angle_x` is the axis `Swerve` actually reads
  through `dc.k_imu_yaw_getter`, the two signals are independent, and OnboardIMU is
  CCW-positive where the old navX was CW-positive.
- **Delete the `reset_position` cheat.** `Swerve.periodic()` already calls
  `pose_estimator.update_with_time(...)` with `get_module_positions()`. Once the modules
  report simulated positions, odometry runs for real.

### Vision
`simulation/vision_sim.py` is **already the right shape** — it fakes the `/Cameras/...`
topics that `vision.py` subscribes to, which is exactly what this model wants. It only needs
an owner.
- Move it into `Vision.simulation_periodic()`.
- The `is_simulation()` branches at `vision.py:103` and `vision.py:219` can probably collapse
  once `vision_sim` owns the fakery.

### Quest
Already self-contained behind `k_mock_questnav` and `is_simulation()`. Nothing to move.

### LED / Targeting / RobotState
No hardware to model. `AddressableLEDSim` exists if we ever want it. Nothing to do.

### BlockheadMech
Already driven from `robot_periodic()` and already works on the real robot. **Leave it** — it
is the one piece already doing it the WPILib way. (Note it is currently invisible on a7 for
an unrelated reason; see §7.)

## 5. Two things that have no subsystem

**Gamepiece sim** (`simulation/gamepiece_sim.py`) is field state, not a mechanism. Put it on
`RobotState` — already a `Subsystem` with a callback bus, and Intake needs to ask it "am I on
a piece?" to drive that DIO switch.

**`_snap_to_live_tags()` and `_snap_to_quest()`** teleport ground truth from *real hardware*
for hardware-in-the-loop testing. These genuinely do not fit the per-subsystem model. Give
them an explicit `HardwareInTheLoop` helper pumped from `MyRobot.simulation_periodic()`.
**Do not lose these** — they are the least replaceable thing in the current sim.

## 6. The one real design decision

Does ground truth survive?

1. **No ground truth at all.** Encoders and IMU are simulated, odometry drifts, vision
   corrects. Most faithful.
2. **Keep it, but only as the source for the vision and gamepiece sims** — a camera should
   see where the robot *is*, not where it thinks it is — and publish it as a separate Field2d
   object so estimator error is visible on screen.

**Recommendation: (2).** It is what the `/Sim/ground_truth` topic already does, and it turns
estimator error from invisible into something you can watch drift.

## 7. Interaction with the a7 Field2d gap — read before starting

The old version of this plan said "publish `Field2d` from `Swerve`, unconditionally, on the
real robot too". That is still the right destination, **but on a7 it does not work the
obvious way.**

`Field2d` and `Mechanism2d` cannot be published through the telemetry registry from Python at
all: both expose `log_to(_NativeTelemetryTable)` and nothing hands Python one. See
`_NATIVE_GAP` in `helpers/dashboard.py`. Both are written to NetworkTables by hand instead
and both work — Field2d in `dashboard.py`, Mechanism2d in `helpers/mechanism_publisher.py`.

So when Swerve takes ownership of the field:
- publish it via `helpers.dashboard`, not `SmartDashboard` (which no longer exists) and not
  `log_to`;
- register it once so `dashboard.update()` re-publishes it each loop — unlike a real Sendable
  it does not update itself;
- re-test `log_to()` on each new alpha. When it stops raising `TypeError`, the hand-written
  publisher can go.

## 8. Order of work

| # | Step | Rough size |
|---|------|-----------|
| 1 | `motors.py` sim seam | an evening |
| 2 | Climber | an evening |
| 3 | Shooter | an evening |
| 4 | Intake (arm + DIO switch) | an evening |
| 5 | Swerve + SwerveModule; delete the odometry cheat | a weekend |
| 6 | Vision + gamepiece rehoming, HIL snap helper | an evening |
| 7 | Delete `physics.py`, `simulation/physics_interface.py`, `simulation/swerve_sim.py` | — |

**This is incremental, not a big bang.** `physics.py` keeps handling whatever has not been
migrated, as long as each piece is removed from `update_sim()` as its subsystem takes over.
The sim stays working at every step.

## 9. How to verify each step

There is no substitute for actually running it, and a7 fails silently in several places:

- `robotpy test` — 12/12 today; keep it there.
- Run the robot headless in sim and **count tracebacks, not just exit code**. Subsystem
  `periodic()` and phoenix6's background threads raise onto a *thread*, so the program keeps
  running and the failure scrolls past. A clean run means zero `Traceback` lines.
- After each subsystem moves, drive it in sim and confirm the value the robot *reads back*
  changes — not just that the plant model ran.

For step 5 specifically: once the odometry cheat is gone, the estimator should visibly
disagree with ground truth over a long drive. **That disagreement is the feature.** If pose
still tracks perfectly, the module sims are not actually feeding the estimator.
