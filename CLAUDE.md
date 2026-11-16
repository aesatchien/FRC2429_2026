# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## House rules - read these before adding anything

These apply to every folder in this repo and to everyone who commits here, with or without an AI
assistant. They exist because features kept arriving as a new file, a new call in `robot_periodic`
and a new protocol, written without reading the code that was already there.

### 1. Find the owner before you write a file

Nearly everything already has a home. Extend it; do not build a second one next to it.

| What you are adding | Where it goes |
|---|---|
| Robot-wide state or health: power, battery, FMS, brownouts, "what is the robot doing" | `subsystems/robot_state.py` |
| Something the drive team should SEE on the robot | an `Indicator` in `subsystems/led.py` |
| A mechanism's sensors, motors, telemetry | that mechanism's subsystem, in its `periodic()` |
| Per-motor amps / RPM | `helpers/utilities.py` `init_motor_monitors()` / `update_motor_monitors()` |
| A value someone wants in the log | publish it to NT from its owner. `DataLogManager` already writes every NT value to the `.wpilog`; that IS the logging. |
| A console message for a condition that can persist | `helpers/condition_log.py` (`ConditionLog`), never a print in a loop |
| Starting / locating log files, the console file, CAN bus health | `helpers/log_start.py`, `helpers/console_log.py`, `helpers/can_monitor.py`. These three are the whole logging layer. Ask before adding a fourth. |
| An NT topic name | built from the prefixes in `constants.py`, never a literal path in two files |
| A number someone may want to change | `constants.py`, in the class of the subsystem that uses it |
| A dashboard widget | an entry in `gui/config.py` `WIDGET_CONFIG` with an existing `update_style`. A new method in `dashboard_2026.py` is the last resort. |
| Anything in `robot.py` | No. `robot.py` starts things. `robot_periodic` / `robotPeriodic` is closed: `tests/test_robot_periodic_is_closed.py` fails if something new is called from it. |

If you cannot find the owner, ask the mentor who owns the repo before inventing one.

### 2. The robot loop is a budget, not a free 50 Hz timer

Every line in a `periodic()` runs 50 times a second, forever, next to the swerve.

- Each block in a `periodic()` says how often it runs: `if self.counter % N == 0:` with the rate in
  the comment. Nothing new runs every loop unless it is closing a control loop.
- Pick the rate from who consumes the result. A person looking at a dashboard needs 2-10 Hz. A
  thing that happens once per boot, or at the speed a human clicks, belongs in an existing 1-2 s
  block.
- Only do it when it can happen. A battery, an alliance color or an auto choice cannot change
  while enabled, so those checks run only while disabled.
- Reuse a block that already runs at the rate you need (the 2 s FMS block in `RobotState`, the
  2 s and 10 s blocks in `disabled_periodic`) instead of adding a counter.
- Every NT value costs log disk as well as CPU. Publish on change or throttled; print events
  once, not states repeatedly.

### 3. The robot and the dashboard already know how to talk

There are three patterns. Use one of them; do not design a fourth.

- **Status**: the robot publishes a topic, a `WIDGET_CONFIG` entry displays it (`indicator`,
  `monitor`, `lcd`).
- **Button**: a command published with `putData`; the dashboard writes `<Command>/running`
  (`command_topic`).
- **Chooser**: the robot publishes `options` (string array) and `active` (what it actually has),
  and reads `selected`; the dashboard side is `update_style: 'combo'`. The robot is the source of
  truth and the dashboard shows `active`. The dashboard closes its `selected` publishers when the
  link drops, so a restarted robot is never handed a stale choice. That problem is solved; do not
  solve it again with ids, tokens or composite strings.

The lists (auto routines, batteries) live on the robot, in `constants.py`. The dashboard has no
copy.

### 4. Small, single-purpose changes

- One feature per commit and per PR. A battery picker, a log-folder change and an analysis tool
  are three PRs.
- Count before you write. A dashboard-to-robot setting should cost about 20 robot lines and one
  config entry. If the plan needs a new module, a new call in `robot.py` and 100+ dashboard lines,
  the plan missed something that already exists; go back to rule 1.
- Do not add configurability, validation layers, fallbacks or protocols for cases nobody has
  seen. Do not rewrite code you were not asked to touch.
- Worked example, the battery picker (2026-11): the first version was a new `helpers/battery_id.py`
  polled every loop from `robot_periodic`, a random boot id with a `"<battery>@<boot id>"` wire
  format over three new topics, and 144 lines of hand-built Qt with its own timer and popup. What
  is in the repo is one `_check_battery()` in `RobotState` on its existing 2 s block, disabled
  only, three chooser-shaped topics, one `WIDGET_CONFIG` entry and a status-bar combo box.

### 5. You are the author of what you submit

An AI assistant may write the code. The person opening the PR must have read every line of the
diff, be able to say why each new file and each new periodic call is needed, and have run it. The
PR template asks for exactly that. A PR whose author cannot answer those questions gets closed,
not reviewed.

If you are the AI assistant: before writing, read the owner file from rule 1 and the nearest
existing example of the same pattern, and say in your summary which existing code you reused and
what now runs per loop and at what rate. If the request cannot be met without breaking a rule
above, stop and say so instead of working around it.

## Repo layout

This is a monorepo of **independent FRC robot projects** (team 2429), each a self-contained
RobotPy app with its own `pyproject.toml`, `robot.py`, `robotcontainer.py`, `constants.py`,
`subsystems/`, `commands/`:

- `comp_bot/` — the **roboRIO** version of the competition robot code. Pinned to
  `robotpy==2026.2.2`. The competition robot itself now runs a SystemCore, so this code no
  longer runs on that robot — but it is **actively maintained, not historical**: the team
  still has many roboRIOs and will for the next year or two, so this is the version every
  roboRIO robot runs. See "Keeping the two versions straight" below.
- `comp_system_core/` — the **SystemCore** version of the same robot code (FRC's new roboRIO
  replacement). Pinned to the `robotpy==2027.0.0a7` alpha. This is what the competition
  robot runs now. It is a parallel codebase, not a shared-code variant of `comp_bot` —
  nothing is shared, so a change made in one is NOT in the other until someone ports it. See
  `comp_system_core/docs/a7_migration.md` before editing anything here.
- `other_robots/` — `practicebot` (swerve test bed), `practice_bot_remake`, `tankbot`
  (west-coast-drive + shooter/turret tutorial bot), `chairbot` (STEAM outreach go-kart),
  `template` (starter skeleton). All pinned to 2026-era robotpy like `comp_bot`.
- `gui/` — a standalone PyQt6 driver/dev dashboard (not Shuffleboard/AdvantageScope — a
  team-built tool). Connects over NT4 to whichever robot project is running. See
  `gui/README.md` for its own architecture (config-driven widgets, `WIDGET_CONFIG` in
  `gui/config.py`).
- `deploy/`, `resources/`, `networktables.json` — shared assets (PathPlanner paths, field
  images, git guidelines).

## Keeping the two versions straight

`comp_bot` (roboRIO) and `comp_system_core` (SystemCore) are two versions of the same robot
code that both have to stay correct until the team has enough SystemCore controllers to
retire the roboRIOs.

- **A feature or fix that applies to both goes in both**, ported by hand: snake_case and the
  other a7 API changes in `comp_system_core`, camelCase 2026 API in `comp_bot`. Say in the
  commit message which project(s) it touches and whether the other still needs it.
- **Hardware differs, so do not copy hardware assumptions across.** On the roboRIO there is
  ONE CAN bus (no CANivore): 4 Krakens, 14 REV controllers and the PDH all share it. On
  SystemCore the buses are split: Krakens alone on `can_s0`, swerve turn SparkFlexes on
  `can_s1`, everything else on `can_s2` (`constants.k_can_bus_*`). The IMU (navX vs
  `wpilib.OnboardIMU`), analog rail (5 V vs 3.3 V) and NT timestamp units also differ.
- **Say where something was tested.** The competition robot is SystemCore, so `comp_bot`
  changes are only hardware-tested on one of the team's other roboRIO robots. If a change
  was only run in `robotpy test`/sim, say so — never write "confirmed on the robot" for a
  project that robot does not run.

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
- **Battery picker**: `RobotState` publishes `<status_prefix>/battery/options` and `/active` and reads
  `/selected` (the chooser pattern), checked every 2 s while disabled. `active` is `?` on every boot
  until someone picks one in the dashboard's status bar. A pick prints `BATTERY: <id>`, the line Scope2429's
  battery report looks for in the console log; the list is `RobotStateConstants.k_batteries`.
- **QuestNav** (headset-based odometry) is also split up:
  - `subsystems/quest.py` — robot-side subsystem: sync/resync state machine, strict pose
    acceptance criteria, lost-stream recovery (soft resync), dashboard publishing.
  - `helpers/quest_double_tap.py` — the double-tap silo, switched by
    `QuestConstants.k_check_for_double_tap`. It only raises `/QuestNav/quest_in_passthrough`
    (the dashboard's cue to run its adb fix, `gui/quest_double_tap.py`) and counts taps.
    Stream-loss detection, unsync, soft resync and auto-resync are NOT double-tap code and
    run either way. The GUI follows the robot's setting over
    `/QuestNav/check_for_double_tap`; it has no flag of its own.
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
  `Velocity`/`Position`/`Acceleration`), published at 10 Hz (`self.counter % 5 == 0`).
  Not every loop: `DataLogManager` writes every NT value to the `.wpilog`, and 50 Hz was
  roughly 25 MB of disk per hour of robot uptime; 10 Hz still animates fine.
  `module_states_desired` is what each module actually COMMANDED (`getCommandedState()`,
  after `optimize()` and the low-speed hold), not the raw request. Every other dashboard
  value is throttled the same way, with `self.counter % N == 0`; before publishing
  anything, ask who reads it and how often they actually need it.
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
