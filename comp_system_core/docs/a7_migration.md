# WPILib 2027.0.0a6 → a7 Migration

Done 2026-09-13 on branch `migrate-2027a7`.

**a7 is not a version bump. It is a rename of the entire Python API plus the deletion of
Sendable.** Read this before touching a8.

---

## 1. What actually changed

| | a6 | a7 |
|---|---|---|
| Naming | camelCase | **snake_case everywhere** — wpilib, ntcore, commands2, robotpy-rev, all at once |
| Compatibility aliases | — | **none** |
| `Sendable` / `SendableRegistry` | yes | **deleted** |
| `SmartDashboard` | yes | **deleted** → `robotpy-telemetry` |
| `SendableChooser` | yes | **deleted** → `tunables.Selectable` |
| `AprilTagFieldLayout` | in robotpy-apriltag | **deleted** — detector/estimator only |
| REV conversion factors | `positionConversionFactor` | **deleted, no replacement** |
| Geometry accessors | `Pose2d.X()` method | `pose.x` **property** (parens must go) |
| REV enum members | `kSlot0` | `SLOT0` (SCREAMING_SNAKE) |
| `hal.initialize(500, 0)` | 2 args | **no args** |

Measured blast radius in this repo: **~1850 call sites across 200 distinct API names**,
1492 in our code and 358 in vendored `pathplannerlib/`.

## 2. How it was done

The rename table was **derived, not hand-written**: extract every API name from the a6 stubs
and the a7 wheels, convert camelCase→snake_case, and keep only pairs where the snake form
actually exists in a7. That produced 1902 confident renames with zero guessing.

Applying it used Python's `tokenize`, not regex, so **string literals and comments are
structurally untouchable** — critical because NetworkTables keys look exactly like method
names. Commented-out code was deliberately left stale rather than risk mangling it.

Two rules the applier had to follow, both learned the hard way:

- **Rename definitions as well as call sites.** a7 renamed the lifecycle hooks, so renaming
  only calls would leave `def robotPeriodic` overriding nothing — a robot that constructs
  fine and then silently does nothing. 116 of the 1389 renames were `def`s.
- **A cross-class name map produces false positives.** `kV` mapped to a `kv` that exists on
  some *other* a7 class; the real answer was `FeedForwardConfig.v`. Likewise
  `nominalVoltage` got renamed at 5 use sites while pathplannerlib's dataclass field kept
  the old spelling. The import loop is what catches these — do not trust the map alone.

## 3. Things no automated rename can catch

**Strings fed to `getattr`.** `swerve_constants.k_imu_yaw_getter = 'getAngleX'` and its
pitch/roll siblings. Invisible to the tokenizer, and getting one wrong loses the gyro axis
at runtime. All three fixed by hand.

**Capitalised accessors.** `IZone` → `i_zone`, `DCMotor.Kt` → `kt`, `Pose2d.X()` → `.x`.
A `[a-z][A-Z]` camelCase pattern misses every one of them.

**Method-vs-property collisions.** After `Pose2d.X()` became the property `.x`, a sweep that
stripped `.x()` parens also hit `CommandXboxController.x()` — the Xbox **X button**, which
really is a method. Result: `'function' object has no attribute 'on_true'`. There is a
warning comment in `helpers/joysticks.py` so it does not happen again.

**Keyword argument names.** `idleMode=` → `idle_mode=`, `Kp=` → `kp=`, `fullRange=` →
`full_range=`. Only rename a kwarg when the snake form is a real a7 parameter *and* the
camelCase form is not a parameter of one of our own functions.

## 4. New files this migration added

| File | Why |
|---|---|
| `helpers/dashboard.py` | SmartDashboard replacement over `TelemetryRegistry` / `TunableRegistry` |
| `helpers/apriltag_layout.py` | our own `AprilTagFieldLayout` — a7 deleted robotpy's |
| `helpers/phoenix6_compat.py` | aliases the camelCase symbols phoenix6 still calls |
| `helpers/mechanism_publisher.py` | writes Mechanism2d to NT by hand — a7 cannot publish one |

### Two non-obvious requirements in `dashboard.py`

1. **Backends must be registered or telemetry is silently discarded.** With none registered
   the registry hands back a `DiscardTelemetryBackend`: `log()` succeeds, prints one
   "no backend for path" warning, and drops the value. `dashboard.install()` runs at import
   time in `robot.py`, before `RobotContainer` publishes anything.
2. **`TunableRegistry.update()` must run every loop** or values never come *back* from the
   dashboard — the auto chooser would never change and no command button would ever fire.
   `dashboard.update()` does this from `robot_periodic()`.

The telemetry/tunable split is real and matters: `publish_tunables()` is what makes a
command button pressable, `log_to()` is read-only. `put_data()` routes on that.

## 5. Still degraded — re-check on every new alpha

**phoenix6 cannot construct a TalonFX on a7 without our shim.** 26.50.0a1 is the newest
build CTRE has published and it is written against the camelCase API. `helpers/phoenix6_compat.py`
aliases the ~20 symbols it needs. Note that **pip resolves phoenix6 against a7 happily** —
it declares no wpilib bound — so a clean `pip install` proves nothing about runtime. The
four HAL sim callbacks it needs for CANcoder/CANdi/CANrange are shimmed too, but that path
is untested because we have no such devices.

**Field2d and Mechanism2d cannot be published from Python at all.** Both expose
`log_to(_NativeTelemetryTable)` and nothing hands Python one:
`TelemetryRegistry.get_table()` returns the Python `TelemetryTable`, and
`_NativeTelemetryTable` reports "No constructor defined!". This is an alpha binding gap.

Both are reproduced by hand instead, so both views work: Field2d directly in `dashboard.py`,
Mechanism2d in `helpers/mechanism_publisher.py`. The Mechanism2d one needs more machinery
because a7 cannot *walk* a mechanism — `Mechanism2d` has only `get_root()` and
`MechanismRoot2d` only `get_name()`, so neither can list its children — so the tree is
recorded as it is built by wrapping the four builder methods. The wire format was captured
from a6 rather than guessed.

When `log_to()` stops raising `TypeError`, delete the `_NATIVE_GAP` section **and**
`helpers/mechanism_publisher.py`.

## 6. NEEDS A BENCH TEST BEFORE DRIVING

rev deleted `positionConversionFactor` / `velocityConversionFactor` with no replacement, so
controllers now report **raw motor rotations and RPM**. The scaling moved into
`subsystems/motors.py`, which is where the Kraken path already did it.

**This changes closed-loop gains.** The PID now sees error in motor units, not metres or
degrees, so gains must be multiplied by the conversion factor to produce the same output for
the same physical error:

- `swerve_constants.k_drive_kp_rev_raw` — only live when `drive_vendor == 'rev'`; the comp
  bot is `'ctre'`, so this is unexercised.
- **`constants.IntakeConstants` deploy loop — this one IS live.** The on-Spark slot gains
  (`k_deploy_config.closed_loop.pid`) now see error in motor rotations, roughly 8× per unit.
  The WPILib `ProfiledPIDController` that actually runs the arm still sees degrees: the sim
  migration found that `intake.py` never applied `k_deploy_position_factor` at all, and now
  funnels every encoder read/write through `Intake.get_angle_deg()` /
  `_set_deploy_angle_deg()`. Verify the deploy holds position on the bench before trusting it.

## 7. Environments

- `robo2027_a7` mamba env — a7 (this branch); it began as the venv `C:\FRC\2026\venv_a7`.
  `robotpy test` → 13/13 (12 plus the end-to-end simulation test added with the sim migration).
- `robo2027_alpha` mamba env — still a6, untouched, so `main` stays deployable.

## 8. What is verified, and what is not

**Verified in sim:** robot imports, constructs and runs with zero tracebacks; `robotpy test`
12/12; Field2d publishes with the correct start pose; dashboard command buttons publish as
tunables; the auto chooser publishes `default`/`options`; physics and the sim rig run.

**Not verified:** anything on real hardware. No Kraken has spun, no Spark has moved, the
deploy gains are arithmetic rather than measured, and the phoenix6 shim has only been
exercised in simulation. Bench it before it drives.
