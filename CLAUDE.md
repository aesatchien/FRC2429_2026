# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repo layout

This is a monorepo of **independent FRC robot projects** (team 2429), each a self-contained
RobotPy app with its own `pyproject.toml`, `robot.py`, `robotcontainer.py`, `constants.py`,
`subsystems/`, `commands/`:

- `comp_bot/` — the 2026-season competition robot. Targets a **roboRIO**. Pinned to
  `robotpy==2026.2.2`.
- `comp_system_core/` — the 2027-season migration of `comp_bot` onto **SystemCore** hardware
  (FRC's new roboRIO replacement). Pinned to the `robotpy==2027.0.0a7` alpha. This is a
  parallel, independently-maintained codebase, not a shared-code variant of `comp_bot` —
  changes to one do not propagate to the other. See `comp_system_core/docs/a7_migration.md`
  before editing anything here.
- `other_robots/` — `practicebot` (swerve test bed), `practice_bot_remake`, `tankbot`
  (west-coast-drive + shooter/turret tutorial bot), `chairbot` (STEAM outreach go-kart),
  `template` (starter skeleton). All pinned to 2026-era robotpy like `comp_bot`.
- `gui/` — a standalone PyQt6 driver/dev dashboard (not Shuffleboard/AdvantageScope — a
  team-built tool). Connects over NT4 to whichever robot project is running. See
  `gui/README.md` for its own architecture (config-driven widgets, `WIDGET_CONFIG` in
  `gui/config.py`).
- `deploy/`, `resources/`, `networktables.json` — shared assets (PathPlanner paths, field
  images, git guidelines).

## Environments — the single most common mistake in this repo

`comp_bot`/`other_robots` and `comp_system_core` need **completely different, incompatible
Python environments**, and running one project's code in the other's environment fails in
non-obvious ways (`ModuleNotFoundError: wpimath.geometry`, `ImportError: cannot import name
'Sendable'`) rather than a clear "wrong robot" error. Always confirm the active conda/venv
environment matches the directory you're in before running `sim`/`deploy`/`test`:

- **2026 projects** (`comp_bot`, everything under `other_robots/`): environment must have
  `robotpy==2026.2.2` and matching `robotpy-*`/`wpilib`/`pyntcore` at the same micro version.
  Uses roboRIO deploy semantics (SSH as `lvuser`, blank password).
- **`comp_system_core`**: environment must have `robotpy[commands2,apriltag,sim]==2027.0.0a7`
  plus the exact pins in its `pyproject.toml`'s `requires=` (notably
  `phoenix6==26.50.0a1` — NOT a looser `>=26.1`, which silently resolves to the 2026 build
  and crashes constructing a `TalonFX`). Deploys to **SystemCore**, not a roboRIO: SSH
  username/password are both `systemcore` (not `lvuser`/blank), target path is
  `/home/systemcore/deploy`.
- If an environment ever ends up with packages from both version lines mixed together (e.g.
  from experimenting with a newer alpha), `pip check` will show the conflicts; fix by
  uninstalling the stray versions and reinstalling the project's pinned `robotpy==` line
  before anything else.

## Commands

Run all of these from inside the specific robot's directory (e.g. `comp_bot/` or
`comp_system_core/`), with that project's environment active.

- **Simulate**: `python -m robotpy sim`
- **Deploy**: `python -m robotpy deploy`
- **Sync dependencies** (installs what `[tool.robotpy]` in `pyproject.toml` declares):
  `python -m robotpy sync`. In `comp_system_core`, use
  `python -m robotpy sync --no-upgrade-project` — plain `sync` will offer to bump the
  alpha version, and `comp_system_core/pyproject.toml`'s header comment is explicit that
  every alpha bump this season has renamed or deleted something; read
  `docs/a7_migration.md` before ever accepting one.
- **Test**: `python -m robotpy test`, not bare `pytest` — in `comp_system_core` especially,
  `pyfrc` is gone in 2027 (`robotpy test`/`robotpy sim` are now built into wpilib core), and
  bare `pytest` skips the deploy-directory setup that
  `pathplannerlib.config.RobotConfig.fromGUISettings()` needs, throwing `FileNotFoundError`
  for every test that touches it. Both `comp_bot/tests/` and `comp_system_core/tests/` exist;
  `comp_system_core`'s is the more complete suite (adds `test_dependency_pins.py`,
  `test_simulation.py`, `test_swerve_optimize.py`, `test_nt_time.py` on top of the
  `test_robot_builds.py` / `test_no_attribute_landmines.py` both share) since the a7
  migration needed a stronger safety net.
- **Lint** (repo-wide, matches CI in `.github/workflows/build.yml`):
  `flake8 . --select=E9,F63,F7,F82` — syntax errors and undefined names only, not style.
- **Dev dashboard**: `python gui/main.py`

## Architecture shared across the robot projects

Each project follows the same shape (`comp_bot` and `comp_system_core` are structurally
near-identical, differing mainly in the a6→a7 API surface — see below):

- `robot.py` → `robotcontainer.py` (`RobotContainer`: instantiates subsystems, binds
  controller buttons, wires the dashboard, builds the autonomous chooser) →
  `subsystems/*.py` / `commands/*.py` / `constants.py`.
- **Swerve drive** is split across three files, each with one job:
  - `subsystems/swerve.py` — chassis-level: pose estimator, odometry fusion (QuestNav +
    AprilTags), dashboard publishing, PathPlanner `AutoBuilder` wiring.
  - `subsystems/swervemodule_2429.py` — one module's state machine (`setDesiredState`,
    optimize, turn-PID-on-the-RIO-against-an-absolute-encoder).
  - `subsystems/motors.py` — the vendor seam. Defines a vendor-neutral `DriveMotor`/
    `TurnMotor` protocol so a module can run REV (SparkMax/Flex) or CTRE (Kraken X60/
    TalonFX) without the rest of the code knowing which. `subsystems/swerve_constants.py`'s
    `ACTIVE_CONFIG` (selected by `constants.k_swerve_config`, e.g. `"comp"` vs
    `"comp_vortex"`) picks the vendor/gearing/CAN-ID set per robot build.
- **QuestNav** (headset-based odometry) is also split in two:
  - `subsystems/quest.py` — robot-side subsystem: sync/resync state machine, strict pose
    acceptance criteria, passthrough/double-tap recovery, dashboard publishing.
  - `helpers/questnav/questnav.py` — low-level NT4 client that parses the Quest headset's
    own protobuf topics (`frameData`, `deviceData`, command `response`) into `PoseFrame`
    objects. Protobuf schema classes are generated under
    `helpers/questnav/protos/generated/`.
- **NetworkTables topic prefixes are centralized** in each project's `constants.py`
  (`swerve_prefix`, `quest_prefix`, `camera_prefix`, `command_prefix`, etc). Not everything
  lives under `/SmartDashboard` — `/QuestNav/...` and `/Cameras/...` are deliberately
  top-level, parallel to `/SmartDashboard`, since they're consumed by AdvantageScope and
  vision coprocessors directly.
- **AdvantageScope swerve visualization**: `<swerve_prefix>/module_states` and
  `module_states_desired` are struct-array topics (`SwerveModuleState[]` on 2026,
  `SwerveModuleVelocity[]` on 2027a7 — a7 split `SwerveModuleState` into
  `Velocity`/`Position`/`Acceleration`), published every `periodic()` tick unthrottled so
  the widget animates smoothly. Most other dashboard values are throttled via a
  `self.counter % N == 0` pattern instead.
- **REV motor current/speed telemetry**: `helpers/utilities.py`'s `init_motor_monitors()` /
  `update_motor_monitors()` give any subsystem a one-line way to publish per-motor amps and
  RPM; Kraken-specific electrical telemetry (stator/supply current, applied voltage) lives
  directly in `swerve.py`/`motors.py` since Phoenix exposes richer signals than REV does.

## `comp_system_core` (2027a7 / SystemCore) specifics

`comp_system_core/pyproject.toml`'s header comment and `docs/a7_migration.md` are the
authoritative references — read them before making non-trivial changes here. The short
version of what differs from `comp_bot`:

- Everything renamed camelCase → snake_case across wpilib, ntcore, commands2, and
  robotpy-rev, with no compatibility aliases (`NetworkTableInstance.getDefault()` →
  `get_default()`, `Pose2d.X()` method → `.x` property, etc).
- `Sendable`/`SendableRegistry`/`SmartDashboard`/`SendableChooser` are deleted outright,
  replaced by `robotpy-telemetry`/`robotpy-tunables`; `helpers/dashboard.py` wraps them
  behind a `SmartDashboard`-shaped API so call sites didn't all need rewriting.
  `dashboard.install()` must run before anything publishes, or telemetry calls silently
  no-op with a "no backend for path" warning.
- `wpimath.geometry`/`.kinematics`/`.controller` submodules are gone — `Pose2d`,
  `SwerveDrive4Kinematics`, `PIDController`, etc. are all flat on `wpimath` directly.
- SystemCore has `wpilib.OnboardIMU` on-board; there is no navX anymore.
- `robotpy_apriltag` lost `AprilTagFieldLayout`; `helpers/apriltag_layout.py` is a local
  replacement.
- `Field2d`/`Mechanism2d` can't be published through the telemetry registry at all yet;
  `helpers/dashboard.py` and `helpers/mechanism_publisher.py` reproduce them by hand.
