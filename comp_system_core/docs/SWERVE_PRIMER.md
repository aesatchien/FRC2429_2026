# Swerve Drive Primer (comp_system_core)

How the drivetrain code is laid out, what runs every loop, and where to change things.
Written against the 2027a7 code in this directory; `comp_bot/docs/SWERVE_PRIMER.md` covers
the roboRIO version (navX, single CAN bus, 2026 API).

## 1. Four files, one job each

| File | Job | Knows about |
| :--- | :--- | :--- |
| `subsystems/swerve.py` | The chassis: gyro, pose estimator, sensor fusion, PathPlanner wiring, dashboard. | four modules |
| `subsystems/swervemodule_2429.py` | One corner: turn a "speed and angle" request into motor commands. | one drive motor, one turn motor, one absolute encoder |
| `subsystems/motors.py` | The vendor seam: makes a REV Spark and a CTRE Kraken look the same. | `rev`, `phoenix6` |
| `subsystems/swerve_constants.py` | Every number: geometry, gearing, gains, limits, CAN ids, which robot build. | - |

`constants.k_swerve_config` (`"comp"`, `"comp_vortex"`, `"practice"`) selects
`DriveConstants.ACTIVE_CONFIG`, which says which vendor is on the drive and turn and which
CAN ids and encoder offsets apply. The comp robot is `"comp"`: Kraken X60 drive (Phoenix,
bus `can_s0`), SparkFlex turn (REV, bus 1).

### The module (`swervemodule_2429.py`)

`setDesiredState(SwerveModuleVelocity)` is the only way in. It:

1. **Optimizes**: if the wheel would have to turn more than 90 degrees, reverse the drive
   instead.
2. **Cosine-scales**: while the wheel is still turning toward its target, drive only as
   fast as the part of the push that points the right way.
3. Holds the wheel where it is if the request is essentially zero speed.
4. Sends the drive motor a speed in **m/s**. The motor controller closes the velocity loop
   (Kraken: `VelocityVoltage` with measured kS / kV).
5. Runs the **turn PID in robot code** against the analog absolute encoder
   (`AnalogPotentiometer`) and sends the turn motor a plain duty cycle.

Step 5 is why the vendor seam is small: a turn motor only has to accept a duty cycle.

In a7 `SwerveModuleState` was split; a module's speed and angle is `SwerveModuleVelocity`,
its distance and angle is `SwerveModulePosition`. `optimize()` and `cosine_scale()`
**return** the new state; calling them without keeping the result does nothing.

### The vendor seam (`motors.py`)

`DriveMotor` and `TurnMotor` are `typing.Protocol`s. There are four implementations
(`RevDriveMotor`, `RevTurnMotor`, `TalonDriveMotor`, `TalonTurnMotor`) and two factories
(`build_drive_motor`, `build_turn_motor`). The contract is **metres and metres per
second**; nothing outside this file knows a gear ratio. The same classes carry the
simulation plant (`sim_update()`).

## 2. What `Swerve.periodic()` does every 20 ms

1. **`_update_vision_measurements()`** - offer the pose estimator anything trustworthy:
   - **AprilTags**: for each tag camera, the 7-number pose on
     `/Cameras/<camera>/poses/tag1`. Skipped if older than 0.5 s, if the tag is not in the
     field layout, if the robot is turning faster than 90 degrees/s, or if the pose is
     outside the field.
   - **QuestNav**: every 4th loop, if the Quest is enabled, synced, connected, its pose is
     accepted (`Questnav.is_pose_accepted()`), within 4 m of the current estimate and
     inside the field.
   - Each measurement goes in with a standard deviation (`constants.DrivetrainConstants.
     k_pose_stdevs_*`); a bigger number means less trust. Tags are trusted more while
     disabled.
2. **`_update_odometry()`** - feed the estimator the gyro angle and the four module
   positions, then clamp the estimate to the field.
3. Publish module states for AdvantageScope at 10 Hz, and the rest of the dashboard values
   (`_update_dashboard()`) at 5 Hz or slower.

The estimator is a `SwerveDrive4PoseEstimator`: wheels and gyro for smooth short-term
motion, vision and the Quest to pull out the drift.

**Heading.** `get_angle()` is the *pose estimate's* heading and is what field-oriented
driving uses. The raw gyro is `wpilib.OnboardIMU` (there is no navX on SystemCore). Which
Euler axis is yaw depends on the WPILib version; it is named in
`DriveConstants.k_imu_yaw_getter`, and the long note there says how to re-measure it.

## 3. Driving: `Swerve.drive()`

```python
swerve.drive(xSpeed, ySpeed, rot, fieldRelative, rate_limited, keep_angle=True)
```

Inputs are **fractions of maximum** (-1 to 1), not m/s. In order:

1. `keep_angle` (if asked): hold the last heading when the driver is not rotating.
2. `rate_limited` (if asked): slew-limit the three inputs (`RateLimiters`, section 2).
3. Scale by `kMaxSpeedMetersPerSecond` / `kMaxAngularSpeed`.
4. If field-relative, rotate into the robot frame using the pose heading.
5. Kinematics: chassis velocities to four module velocities.
6. Desaturate: if any wheel is asked for more than `kMaxTotalSpeed`, scale all four down
   together.
7. `setDesiredState()` on each module.

PathPlanner does not use `drive()`; it calls `drive_robot_relative()` with real m/s.

**Who uses which options.** This matters when tuning, because the limits are in different
places:

| Caller | Slew limiting | keep_angle |
| :--- | :--- | :--- |
| `DriveByJoystickSubsystemTargeting` (teleop default) | its own limiters; `rate_limited=False` | off |
| `DriveToPoseCustomControl` | its own limiters; `rate_limited=False` | off |
| `DriveByVelocitySwerve` | `rate_limited=True` | on when field-relative |

So in a match, `keep_angle` and the `default_*_slew_rate` limiters inside `swerve.py` are
not in play. The driver's "feel" is in the joystick command: deadband, response curve,
slow-mode floor, turbo trigger, translation slew.

## 4. Constants you will actually change

All in `subsystems/swerve_constants.py` unless noted. The header comment of that file maps
the four control layers (driver input, targeting, kinematics, motor).

| Constant | What it is |
| :--- | :--- |
| `DriveConstants.kMaxSpeedMetersPerSecond` | The speed full stick asks for. |
| `DriveConstants.kMaxAngularSpeed` | The rotation rate full stick asks for, rad/s. |
| `DriveConstants.kMaxTotalSpeed` | Per-wheel ceiling for desaturation; derived from the drive motor's free speed. |
| `DriveConstants.kSlowModeCap`, `kAngularSlowFloor` | Fraction of max with the turbo trigger released. |
| `RateLimiters.*` | Every slew rate, with a note on which command uses it. |
| `ModuleConstants.kTurningP` | Stiffness of the wheel-angle loop. |
| `ModuleConstants.k_kraken_kv`, `k_kraken_ks`, `k_kraken_kp` | Drive velocity loop (measured by SysId; `tests/test_kraken_gains.py` pins them). |
| `ModuleConstants.kDrivingMotorCurrentLimit`, `k_kraken_stator_current_limit` | Supply and stator limits. Brownout mode lowers the supply limit. |
| `DriveConstants.COMP_CONFIG['modules']` | CAN ids, encoder ports and `turning_offset` per corner. |
| `AutoConstantsSwerve.*` | PathPlanner PID and constraints; drive-to-pose tolerances. |
| `TargetingConstants.*` | Drive-to-pose PIDs and hub-tracking gains. |
| `constants.DrivetrainConstants.k_pose_stdevs_*` (in `constants.py`) | Trust in vision. |

`deploy/pathplanner/settings.json` is edited in the PathPlanner GUI and is not linked to
these. `Swerve._check_pathplanner_config()` prints a loud banner at boot if they disagree.

## 5. Simulation

Simulation lives inside the subsystems (see `docs/sim_migration_plan.md`).

- **`SwerveModule.simulation_periodic()`**: each module advances a plant model for its
  drive and turn motor (`motors.py`, `sim_update()`), writes the result into the
  controller's own sim state, and writes the wheel angle back into the analog encoder. So
  the real code path reads simulated motion.
- **`Swerve.simulation_periodic()`**: pumps the modules, drives `OnboardIMUSim` from what
  the wheels actually did, sags the simulated battery, and integrates a **ground truth**
  pose, published on `/SmartDashboard/Sim/ground_truth` and drawn as `GroundTruth`.
- **Odometry is real.** The estimator is never reset to ground truth, so it can drift and
  disagree with vision as it does on carpet. `Robot` is the estimate, `GroundTruth` is
  where the robot is; the dashboard commands `SimKickRobot` / `SimKickQuest` push them
  apart on purpose so you can watch the estimate recover.
- **Real sensors against the sim**: `simulation/hil_snap.py`, pumped from
  `MyRobot.simulation_periodic()`. With `SimConstants.k_snap_to_real_sensors`, a real
  camera's tag pose or a real synced Quest teleports *ground truth*. It never touches the
  estimator.

## 6. How to...

- **Re-zero a wheel**: set `DriveConstants.analog_encoder_test_mode = True`, point the
  wheels straight, read `absolute_LF` ... `absolute_RB` under `/SmartDashboard/Swerve`,
  put them in `turning_offset`, set the flag back. Do not drive in test mode.
- **Swap a drive motor vendor**: change `constants.k_swerve_config`; fix `driveMotorType`
  in the PathPlanner GUI when the boot banner tells you to. The REV drive path is tested
  in simulation only (`tests/test_rev_drive_gains.py`).
- **Add a value to the dashboard**: create the publisher in `_init_networktables()`, set it
  in `_update_dashboard()` under a `self.counter % N` gate. Ask who reads it and how often
  before picking N; everything on NT is also written to the `.wpilog`.
- **Check motor faults**: run the `CANStatus` dashboard command while disabled. Do not
  poll Kraken faults from `periodic()`; the note in `_update_dashboard()` says why.

## 7. Troubleshooting

- **Field-oriented driving is rotated**: the pose heading is wrong. Driver Y resets the
  pose (see `docs/CONTROLLERS.md` for exactly what it resets to). If it comes back wrong,
  compare `_imu_raw_angle` with `drive_theta` to see whether the gyro or vision is at fault.
- **Pose stopped turning with the robot after an upgrade**: `k_imu_yaw_getter` is pointing
  at the wrong Euler axis. Spin the robot and watch `_imu_anglex/y/z`.
- **A wheel points the wrong way or hunts**: its `turning_offset`, or the analog rail
  (3.3 V on SystemCore, 5 V on a roboRIO).
- **Robot slides off line in sharp turns at full stick**: look at `module_states` against
  `module_states_desired` in AdvantageScope; `kMaxTotalSpeed` is the limit being hit.
- **Path following is off but teleop is fine**: check for the PathPlanner settings banner
  at boot.
- **Drive-to-pose stalls or wanders**: set `print_debug = True` in
  `DriveToPoseCustomControl` and read `docs/DRIVE_TO_POSE_HOWTO.md`.
