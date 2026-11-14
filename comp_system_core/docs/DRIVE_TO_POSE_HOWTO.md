# How to drive to a pose: `DriveToPoseCustomControl`

This is the one command for "drive the robot to this spot and stop there". It replaces
`AutoToPoseClean`, which is retired (see the last section).

This copy is written for **comp_system_core** (2027a7, snake_case). `comp_bot/docs` has the
same howto in the 2026 camelCase API. The command behaves identically in both projects.

- Source: `commands/drive_to_pose_custom_control.py`
- Proven in sim by: `tests/test_simulation.py` (drives onto a simulated game piece)

## The one idea

You do not give the command a pose. You give it a **function that returns a pose**:

```python
DriveToPoseCustomControl(container, swerve, target_pose_supplier=<function returning Pose2d or None>)
```

The command calls that function **once, when it starts** (in `initialize()`, not when the
command object is built). Whatever comes back is the goal for the whole run.

That is why one command covers every case. A fixed spot, an alliance-mirrored spot, a spot
reflected for the top or bottom of the field, a game piece a camera can see right now: they
are all just different functions. The command neither knows nor cares which.

Two consequences worth remembering:

- **Late answers are fine.** Alliance colour, current pose and camera targets are read at
  start time, so a command built at boot (before the FMS has told us the alliance) still
  gets the right answer.
- **Return `None` to cancel.** If the function returns `None` the command ends immediately
  and flashes the failure LED. Use that for "no target in view".

## Arguments

| Argument | Meaning |
|---|---|
| `container` | The `RobotContainer` (used for the LEDs and the swerve). |
| `swerve` | The swerve subsystem. The command requires it, so it interrupts driving. |
| `target_pose_supplier` | Function with no arguments returning a field `Pose2d`, or `None` to abort. |
| `tolerance_type` | `'exact'` (default) or `'fast'`, see below. |
| `indent` | Only affects console log indentation inside auto groups. |

The pose is always a **field pose in blue-origin coordinates**: x along the field from the
blue wall, y to the left, rotation counter-clockwise. The robot ends up at that x, y **and
heading**.

## Recipes

### 1. A fixed spot on the field

```python
from wpimath import Pose2d, Rotation2d
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

spot = Pose2d(3.0, 2.3, Rotation2d.from_degrees(44))
DriveToPoseCustomControl(self, self.swerve, lambda: spot)
```

This goes to that literal spot whichever alliance we are on.

### 2. A blue-side spot that should mirror when we are red

```python
from helpers.apriltag_utils import mirror_for_alliance

DriveToPoseCustomControl(self, self.swerve, lambda: mirror_for_alliance(spot))
```

`mirror_for_alliance` rotates the pose 180 degrees about field centre when we are red. The
old `AutoToPoseClean` did this for you silently; here you say it, which is the point.

### 3. An auto play that works from the top or bottom of the field

This is what the autos use. `auto_reflect_pose` picks the alliance side and also flips the
play about the field's centreline depending on which half the robot is in.

```python
import wpilib
from constants import AutoConstants as ac
from helpers.apriltag_utils import auto_reflect_pose

self.add_commands(
    DriveToPoseCustomControl(
        container=self.container, swerve=self.container.swerve,
        target_pose_supplier=lambda: auto_reflect_pose(
            self.container.swerve.get_pose(), ac.k_first_ball_pickup_pose,
            wpilib.MatchState.get_alliance(), is_shooting=False),
        tolerance_type='fast',
    ).with_timeout(5))
```

Pass `is_shooting=True` for a shooting pose, so the heading is flipped to face the hub.

### 4. A game piece (or anything else) a camera can see

```python
DriveToPoseCustomControl(
    self, self.swerve,
    lambda: self.vision.get_vision_target_pose(self.swerve.get_pose(), ['logi_front_hsv']))
```

`Vision.get_vision_target_pose(current_pose, cameras, offset)` is the one place a camera's
"distance and angle" becomes a field pose: it takes the nearest target on the listed
cameras, rotates by the camera's mount angle and adds it to the robot's current pose. It
returns `None` when nothing is in view, which cancels the command.

- `cameras` is a list of keys from `constants.CameraConstants.k_cameras`. `None` means all.
- `offset` is an optional `Transform2d` in the robot frame, for example to stop short:
  `Transform2d(-0.4, 0, Rotation2d())` stops 0.4 m before the target.

The target is **latched at the start**. The robot does not re-aim if the piece moves, and it
does not care if the piece leaves the camera's view on the way in.

### 5. On a button

```python
js.driver_right.debounce(0.1).while_true(
    DriveToPoseCustomControl(self, self.swerve,
        lambda: self.vision.get_vision_target_pose(self.swerve.get_pose(), ['logi_front_hsv'])))
```

`while_true` means: start on press, cancel on release. Each new press calls the supplier
again, so each press picks a fresh target.

## What it does while running

1. Reads the pose estimate (`swerve.get_pose()`).
2. Runs three PID loops: field x, field y, heading. Gains live in
   `subsystems/swerve_constants.py` (`TargetingConstants.kAutoTranslationPID`,
   `kAutoRotationPID`).
3. Shapes the outputs, in this order:
   - **Stiction floor:** if an axis is still outside tolerance and has not crossed its
     target yet, the output is held to at least 0.1 so the robot does not stall short.
   - **Clamp:** translation to 0.45 of max speed, rotation to 0.5.
   - **Slew limit:** `RateLimiters.auto_translation_slew_rate` and
     `auto_rotation_slew_rate`, so a big error at the start does not brown out the robot.
4. Calls `swerve.drive(..., fieldRelative=True, rate_limited=False, keep_angle=False)`.

"Crossed its target" is detected by the error changing sign. Once an axis has crossed, the
stiction floor is dropped for that axis so the robot settles instead of buzzing back and
forth.

## When it finishes

| `tolerance_type` | Finishes when | Use for |
|---|---|---|
| `'exact'` | within 2 cm **and** 2 degrees for more than 10 loops in a row (about 0.2 s) | shooting poses, anything precise |
| `'fast'` | within 10 cm **and** 10 degrees, immediately | waypoints, driving over game pieces |

The exact tolerances are `AutoConstantsSwerve.k_translation_tolerance_meters` and
`k_rotation_tolerance`.

It also finishes at once if the supplier returned `None`.

**Always add `.with_timeout(...)` in an auto.** A robot pushed against something will never
reach 2 cm, and without a timeout the rest of the auto never runs.

## What you see

- **LEDs:** polka pattern while driving, success flash on arrival, failure flash if it was
  interrupted or had no target.
- **Sim:** the goal is published to `<auto_prefix>/goal_pose` and drawn as the ghost robot
  on the field, so you can see where it thinks it is going before it gets there.
- **Tuning:** set `self.print_debug = True` in the command for a console trace 5 times a
  second. In sim, setpoint and measured values also go to NetworkTables under `auto_prefix`.

## Mistakes to avoid

- **Passing a pose instead of a function.** `DriveToPoseCustomControl(self, swerve, spot)`
  fails when the command starts with "Pose2d object is not callable". Write
  `lambda: spot`.
- **Lambdas in a loop.** `for p in poses: DriveToPoseCustomControl(..., lambda: p)` makes
  every command go to the *last* pose. Write `lambda p=p: p`.
- **Forgetting the alliance.** The command does no mirroring. If the pose was written for
  blue, wrap it (recipe 2 or 3).
- **Expecting it to chase.** It latches one target. To follow something that moves, use a
  tracking command, not this one.
- **No timeout in an auto.** See above.

## Trying it in the simulator

1. `constants.SimConstants.k_disable_vision_sim` must be `False` (it is by default). When
   it is `True` the camera sim publishes no targets and every vision drive cancels.
2. From this directory, with the `robo2027_alpha` environment active:
   `python -m robotpy sim`
3. Enable teleop and drive to within 4 m of the game pieces. They start at x = 7.5 m; the
   robot starts about 4.7 m away, so drive forward a little first.
4. Hold D-pad right (front camera) or D-pad left (left camera). These bindings exist only
   in simulation.

Headless, the same thing runs as part of `python -m robotpy test`.

## `AutoToPoseClean` is retired

Nothing calls `AutoToPoseClean` any more. The file is kept for reference in
`commands/deprecated/auto_to_pose_clean.py`; do not import it in new code. The same was done
in both `comp_bot` and `comp_system_core`.

If you find an old call (in a branch, or in `other_robots/`, which still has its own
copies), this is the translation:

| Old | New supplier |
|---|---|
| `mode="ball_pickup"` | `lambda: auto_reflect_pose(swerve.get_pose(), ac.k_first_ball_pickup_pose, wpilib.MatchState.get_alliance(), is_shooting=False)` |
| `mode="ball_pickup++"` | same with `ac.k_second_ball_pickup_pose` |
| `mode="shooting"` | same with `ac.k_shooting_pose` and `is_shooting=True` |
| `use_vision=True, cameras=[...]` | `lambda: vision.get_vision_target_pose(swerve.get_pose(), [...])` |
| `target_pose=p` | `lambda: mirror_for_alliance(p)` |

Drop `target_pose=None` and `control_type=...`; keep `tolerance_type` and the timeout.

**The autos that were migrated have not been re-run on a robot.** `auto_shoot_and_pickup`,
`twocycle` and `through_trench_pickup` switched commands in November 2026 and were checked
in simulation only. The two commands differ in three ways, so expect to re-tune:

| | `AutoToPoseClean` (old) | `DriveToPoseCustomControl` (now) |
|---|---|---|
| Overshoot detection | error got *bigger* than last loop | error changed *sign* |
| Translation clamp | 0.40 | 0.45 |
| `'fast'` finish | heading in tolerance, and position in tolerance or both axes overshot | within 10 cm and 10 degrees |

The first is the real improvement: the old test tripped on any odometry jump (a vision or
QuestNav correction mid-drive), which removed the stiction floor and could leave the robot
stalled short of the target. The `'fast'` change is the one most likely to alter timing in
`through_trench_pickup`: the new rule ends the move sooner and further from the target.

`AutoToPoseClean` also had a `control_type='pathplanner'` mode. Nothing used it, and the
new command does not have it.
