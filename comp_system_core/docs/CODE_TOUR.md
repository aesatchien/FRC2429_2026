# How to read and edit this code (comp_system_core)

Start here. This is the map; the primers next to it go deep on one area each.

| Read this | When you are working on |
| :--- | :--- |
| `CODE_TOUR.md` (this file) | anything, first |
| `CONTROLLERS.md` | what a button does |
| `SWERVE_PRIMER.md` | the drivetrain, odometry, sensor fusion |
| `DRIVE_TO_POSE_HOWTO.md` | driving to a spot, in teleop or auto |
| `VISION_PRIMER.md` | cameras, game pieces, the simulator's sensors |
| `LED_PRIMER.md` | the LED strip |
| `MATH_PRIMER.md` | poses, rotations, field vs robot frame |
| `a7_migration.md` | **before any change that touches a WPILib / REV / Phoenix call** |
| `sim_migration_plan.md` | how the simulator is built |
| `quest_java.md`, `ORANGE_PI_ML_MODELS.md` | the headset app and the vision coprocessors (code outside this repo) |

## 1. Before you run anything

This project needs the `robo2027_alpha` environment (`robotpy==2027.0.0a7`). The 2026
projects (`comp_bot`, `other_robots`) need a different one, and the wrong environment
fails with confusing import errors, not a clear message. If unsure:

```bash
python -c "import wpilib; print(wpilib.__version__)"
```

From inside `comp_system_core/`:

| To | Run |
| :--- | :--- |
| simulate | `python -m robotpy sim` |
| test (do this before every deploy) | `python -m robotpy test` |
| deploy to the SystemCore | `python -m robotpy deploy` |
| install what `pyproject.toml` pins | `python -m robotpy sync --no-upgrade-project` |

Use `robotpy test`, not bare `pytest`. Never accept an offer to bump the alpha version
without reading `a7_migration.md`.

## 2. The files, in the order to read them

```
robot.py              the program: mode hooks, the per-loop housekeeping
robotcontainer.py     builds every subsystem, binds every button, fills the auto chooser
constants.py          every number that is not about the swerve
subsystems/           one file per mechanism (it owns the hardware and the state)
commands/             one file per action (it tells subsystems what to do)
autonomous/           autos: command groups built from commands and PathPlanner paths
helpers/              shared code that is not a subsystem or a command
simulation/           the parts of the simulator that are not a mechanism
tests/                the safety net - each file's docstring says which bug it exists for
deploy/pathplanner/   paths and settings, edited in the PathPlanner GUI
pathplannerlib/       a VENDORED copy of the library, ported to a7 - see VENDORED.md
commands/deprecated/  retired commands kept for reference; nothing imports them
```

Read `robot.py`, then `robotcontainer.py`, then one subsystem end to end (`shooter.py` is
the simplest real one), then one command that uses it (`commands/shooting_command.py`).
After that the rest is the same shapes repeated.

### Subsystems

| File | Owns | Notes |
| :--- | :--- | :--- |
| `swerve.py` (+ `swervemodule_2429.py`, `motors.py`, `swerve_constants.py`) | drivetrain, gyro, pose estimate, field view | `SWERVE_PRIMER.md` |
| `quest.py` | the QuestNav headset: sync state, pose acceptance, stream-loss recovery | `Swerve` is handed this object and asks it for poses |
| `vision.py` | reads `/Cameras/...` game-piece targets from NetworkTables | tag *poses* are read by `Swerve`, not here |
| `targeting.py` | no hardware: hub aim angle, shot distance, shooter RPM | runs every loop whether or not anyone is tracking |
| `shooter.py` | flywheel, roller, indexer, hopper | |
| `intake.py` | intake rollers and the deploy arm (profiled PID + arm feedforward in `periodic()`) | |
| `led.py` | LED strip | `LED_PRIMER.md` |
| `robot_state.py` | battery / PDH telemetry, FMS state, the simulated game pieces | |
| `climber.py` | nothing: it is not constructed (commented out in `RobotContainer`) and is half written | |

## 3. What happens at boot

1. `robot.py` is imported. `phoenix6_compat` is imported **first** (Phoenix still expects
   the old API names), then `dashboard.install()` runs so telemetry has somewhere to go.
2. `MyRobot.__init__()` (there is no `robotInit()` in 2027) starts console capture and the
   deferred log start, then builds `RobotContainer`.
3. `RobotContainer.__init__()`:
   - constructs the subsystems. Each one configures its motors from `constants.py` and
     creates its NetworkTables publishers in `_init_networktables()`.
   - `bind_driver_buttons()`, `bind_bbox_buttons()`: every button.
   - sets the swerve's default command, `DriveByJoystickSubsystemTargeting`.
   - `register_commands()`: names PathPlanner can call.
   - `initialize_dashboard()`: dashboard command buttons and the auto chooser. **Every
     auto in the chooser is constructed here**, so a broken auto stops the robot at boot,
     and `robotpy test` catches it.

## 4. What happens every 20 ms

`TimedCommandRobot` runs the command scheduler, which for each loop:

1. polls the buttons and starts / cancels commands,
2. calls `execute()` on every running command,
3. calls `periodic()` on every subsystem (and `simulation_periodic()` in the simulator).

Then `robot_periodic()` does the housekeeping: log start, CAN monitor, `dashboard.update()`
(without it nothing comes *back* from the dashboard), and the mechanism drawing.

Two rules follow from this:

- **A subsystem's `periodic()` runs always**, enabled or not. Put sensing and publishing
  there. Put "do something" in a command.
- **Only one command at a time may require a subsystem.** A new command that requires the
  swerve interrupts whatever was driving. A command group requires everything its members
  require, for its whole length.

## 5. Conventions you will see everywhere

- **`self.counter` and `% N`.** Each subsystem counts loops and publishes with
  `if self.counter % 10 == 0:` (5 Hz), `% 50` (1 Hz) and so on. Everything on NetworkTables
  is also written to the `.wpilog`, so publishing every loop costs disk and bandwidth.
  Before publishing, ask who reads it and how often. The counters start at different
  offsets (`k_counter_offset`) so subsystems do not all publish on the same loop.
- **Publishers are created once**, in `_init_networktables()`, and reused. Do not build a
  topic name inside `periodic()`.
- **NetworkTables prefixes come from `constants.py`** (`swerve_prefix`, `quest_prefix`,
  `auto_prefix` ...). Do not type `/SmartDashboard/...` in a subsystem.
- **`@log_command`** on a command class prints a timestamped start / end line. Set
  `self.extra_log_info` in `initialize()` to add detail; `indent` nests the lines inside an
  auto.
- **Motor configs live in `constants.py`** as REV config objects built at import time;
  the subsystem pairs each motor with its config in `configure_sparks([...])`.
- **Units are in the name or the comment**: `_mps`, `_rad`, `_deg`, `rpm`. Swerve drive
  motors speak metres; `Swerve.drive()` takes fractions of max, not m/s.
- **Naming is mixed on purpose-ish.** WPILib calls are snake_case (a7). Some of our own
  older methods are still camelCase (`setDesiredState`, `resetOdometry`, `getState`).
  Match the file you are in; do not rename them as a drive-by.
- **Comments say why**, and usually which bug taught us. Read them before deleting a line
  that looks redundant. When you change a number, do not restate it in a comment
  elsewhere; the restated copy goes stale.

## 6. Recipes

### Bind a button
In `robotcontainer.py`, `bind_driver_buttons()` or `bind_bbox_buttons()`:

```python
js.driver_x.on_true(Intake_Deploy(intake=self.intake, position='up'))       # press
js.driver_x.while_true(SwerveSetX(container=self, swerve=self.swerve))      # hold
js.bbox_1_5.on_true(InstantCommand(lambda: self.questnav.quest_unsync_odometry()).ignoring_disable(True))
```

The names (`js.driver_x`, `js.bbox_1_5`) are defined in `helpers/joysticks.py`, the only
place a raw button number may appear. Check `CONTROLLERS.md` that the button is free, and
update that table in the same commit.

### Write a command
Copy `commands/_logged_command_template.py`. Fill in:

- `__init__`: store what you need, `self.add_requirements(subsystem)`.
- `initialize()`: once, when it starts.
- `execute()`: every loop while it runs.
- `is_finished()`: `True` ends it (one-shot commands return `True`).
- `end(interrupted)`: leave the mechanism the way the next step needs it. Motors keep
  their last command after a command ends, so either stop them here or leave them running
  on purpose and say so (`DriveToPoseCustomControl` does the second by default, so an auto
  can hand off on the move; `stop_at_end=True` makes it stop).

For a one-liner, `InstantCommand(lambda: ...)` in `robotcontainer.py` is enough.

### Put a command on the dashboard
In `initialize_dashboard()`:

```python
dashboard.SmartDashboard.put_data(f'{command_prefix}/HopperOn',
    InstantCommand(lambda: self.shooter.set_hopper_rpm(sc.k_hopper_rpm)))
```

Add `.ignoring_disable(True)` if it must work while disabled. To show it in the team's
PyQt dashboard, add the name to `gui/config.py`.

### Add an auto
1. Copy `autonomous/_commandgroup_template.py`, or an existing auto close to what you want.
2. Build it from `AutoBuilder.followPath(PathPlannerPath.fromPathFile('Name'))` (paths are
   drawn in the PathPlanner GUI and live in `deploy/pathplanner/paths/`),
   `shoot_cycle(container)` from `autonomous/shoot_cycle.py`, and ordinary commands.
   (`followPath` / `fromPathFile` are camelCase because `pathplannerlib/` is our vendored
   copy.)
3. Add it to the chooser in `initialize_dashboard()`:
   `self.auto_chooser.add('5a: My Auto *CODE*', MyAuto(self, indent=0))`.
4. Give every driving step a timeout.
5. `python -m robotpy test`, then run it in the simulator, then on carpet.

A command PathPlanner triggers by name must be registered in `register_commands()`.

### Add or change a constant
Mechanism numbers: the matching class in `constants.py`. Swerve numbers:
`subsystems/swerve_constants.py`. Import it the way the neighbouring code does
(`from constants import ShooterConstants as sc`). The shooter's distance-to-RPM table is
`ShooterConstants.k_distance_to_rpm`.

### Publish a new value
```python
# in _init_networktables()
self.my_value_pub = self.inst.get_double_topic(f"{constants.intake_prefix}/my_value").publish()
# in periodic()
if self.counter % 10 == 0:
    self.my_value_pub.set(value)
```

### Add a motor to a subsystem
1. CAN id and a config object in the subsystem's class in `constants.py`. Check the id is
   free and the device is on the bus you name (`constants.k_can_bus_*`).
2. Construct it in the subsystem, add it to the `configure_sparks([...])` list and to
   `init_motor_monitors(...)` for current / RPM telemetry.
3. Add it to `simulation_periodic()` so the simulator and the tests exercise it.
4. Anything on a GPIO port: SystemCore has six (0 to 5). The map is at the top of
   `constants.py`.

### Add a subsystem
Construct it in `RobotContainer.__init__()`, give it a `k_counter_offset` in `constants.py`,
follow the shape of `shooter.py` (`__init__`, `_init_networktables`, setters, `periodic`,
`simulation_periodic`).

## 7. The safety net

`python -m robotpy test` builds the whole robot in simulation and runs about 70 tests in
under a minute. The ones that matter most when editing:

| Test | Catches |
| :--- | :--- |
| `test_robot_builds.py` | anything that would stop the program at boot: a bad import, a renamed attribute, a broken auto in the chooser, a GPIO port out of range, PathPlanner settings that disagree with the constants |
| `test_simulation.py` | the robot enabled and driving end to end, including a drive onto a game piece |
| `test_no_attribute_landmines.py`, `test_a7_no_camelcase_calls.py`, `test_a7_snake_case_strings.py` | calls to names that do not exist in a7 |
| `test_dependency_pins.py` | an environment that does not match `pyproject.toml` |
| `test_nt_time.py`, `test_swerve_optimize.py`, `test_kraken_gains.py`, `test_rev_drive_gains.py` | one specific bug each; the docstring tells the story |

A test passing means the code runs in simulation. It says nothing about wiring, CAN, or
tuning. Say where a change was tested: `robotpy test`, the simulator, or the robot.

## 8. Things that will bite you

- **Silent failures in a7.** A lifecycle hook with the old name (`robotInit`, `testInit`)
  is never called. A method that used to change an object in place may now return a new
  one (`optimize()`). An input the Driver Station is not reporting reads as zero forever.
  None of these raise. `a7_migration.md` lists the ones we have hit.
- **Raw buttons count from 0** in a7 (from 1 in `comp_bot`).
- **NetworkTables timestamps are nanoseconds.** Use `helpers/nt_time.py`; never do
  arithmetic on a raw NT time.
- **`dashboard.install()` before anything publishes; `dashboard.update()` every loop.**
- **Strings passed to `getattr()`** (`k_imu_yaw_getter`) are invisible to any rename tool.
- **A command group owns its subsystems for its whole length**, so the default drive
  command does not run between the steps of an auto. A driving command that does not stop
  the wheels in `end()` leaves them running into the next step: useful for a moving
  hand-off, a surprise if the next step is a shot.
- **`Targeting` runs all the time** and `robot_state` / `led` indicators persist until
  replaced. "Why is it still doing that" is usually state nobody cleared.

## 9. Keeping `comp_bot` in step

`comp_bot` is the roboRIO version of this robot and shares no code with this directory. A
fix that applies to both has to be made in both, by hand: camelCase and the 2026 API
there, snake_case here. Hardware differs (one CAN bus and a navX there; split buses and
the onboard IMU here), so port logic, not hardware assumptions. Say in the commit message
which project a change touches and where it was tested.
