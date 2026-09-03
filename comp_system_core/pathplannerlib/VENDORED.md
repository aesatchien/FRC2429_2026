# This directory is vendored code.  It is not ours.

`pathplannerlib` is PathPlannerLib's **Python** implementation, copied into this repo
because there is no 2027 release of it on PyPI yet.  Delete this whole directory and go
back to `pip install robotpy-pathplannerlib` the moment RobotPy ships a 2027 wheel.

## Where it came from

    repo    https://github.com/mjansen4857/pathplanner
    branch  2027
    commit  61f2fad  (2026-07-02, "fix build workflow")
    path    pathplannerlib-python/pathplannerlib

Note what that branch actually contains.  Its **Java/C++** side is fully ported to
WPILib 2027 (`build.gradle` pins `wpilibVersion = "2027.0.0-alpha-6"`).  Its **Python**
side is untouched - byte-identical to the released 2026.1.2 apart from 4 lines in
path.py.  So the Java told us exactly what the answers should be, and we applied them
to the Python by hand.

## What we changed, and why each one is what it is

Everything below matches what upstream's own 2027 Java already does, so when their
Python port lands it should look like this.

* wpimath was flattened - `wpimath.geometry` / `.kinematics` / `.controller` /
  `.system.plant` are all just `wpimath` now.  `wpilib.event` folded into `wpilib`.
* `ChassisSpeeds` -> `ChassisVelocities`, `SwerveModuleState` -> `SwerveModuleVelocity`,
  and the `.speed` field -> `.velocity`.  Java: `new DifferentialDriveWheelVelocities(
  states[0].velocity, states[1].velocity)`.
* `ChassisSpeeds.fromFieldRelativeSpeeds(...)` is gone.  Java writes
  `new ChassisVelocities(x, y, w).toRobotRelative(pose.getRotation())`, and
  `fromRobotRelativeSpeeds` becomes `.toFieldRelative(...)`.  Both directions were
  checked numerically against 2026 before being applied, not assumed.
* `hal.report(tResourceType.kResourceType_X, n)` -> `hal.reportUsage("PathPlanner/X", n, "")`.
  Java: `HAL.reportUsage("PathPlanner/PathPlannerAuto", instances, "")`.
* `ChassisVelocities.discretize` went from a static `(vx, vy, omega, dt)` to an instance
  method `(dt)` returning a new object.

## One upstream bug fixed in passing

`util/swerve.py` had `ret_speeds.discretize(ret_speeds, dt)` and threw the return value
away, so the discretization the docstring promises never actually happened.  The Java
line is `retSpeeds = retSpeeds.discretize(dt);`.  Ours now assigns the result.

## One place we knowingly diverge

`RobotConfig`'s own methods got renamed along with everything else, so they are
`toChassisVelocities` / `toSwerveModuleVelocities` here, while the Java kept
`toChassisSpeeds` / `toSwerveModuleStates`.  Nothing in our robot code calls them - we
only use `RobotConfig.fromGUISettings()` and `.moduleConfig` - so this is internal, but
it is a real difference if you ever diff against upstream.

## If you re-vendor

Copy the new source in and re-apply the list above; do not try to merge.  Then re-run
the check that matters: build RobotContainer and generate a trajectory from a real path
file, and confirm peak module speed stays under the module limit.
