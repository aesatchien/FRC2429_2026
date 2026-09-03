"""
The autonomous shooting phase, in one place.

WHY THIS FILE EXISTS
--------------------
Before this, 14 auto files contained 22 hand-copied versions of the same block:

    InstantCommand(start_tracking)
    ParallelRaceGroup(
        ShootingCommand(...),
        DriveByJoystickSubsystemTargeting(...),      # so *something* owns the swerve
        SequentialCommandGroup(wait, shoot angle, 500 rpm, wait, shoot2 angle, wait),
    )
    InstantCommand(stop_tracking)

Clustering them showed 9 "different" versions - but the differences were an auto_timeout of
4.0 / 5 / 3.5, delay_cycles of 10 or default, whether the intake sequence was attached, one
stray trailing comma and one stray line break.  That is not nine designs.  That is one design
and 22 transcription attempts, and the transcription errors are invisible until a match.

The cost of that duplication is not the line count, it is that a fix has to be applied 22
times.  Case in point: every one of those copies reads the driver's joystick during
autonomous (see RotateToTarget's docstring).  With this file, switching all of them to the
safe aim command is ONE edit - change the default of `aim` below.

USAGE
-----
    from autonomous.shoot_cycle import shoot_cycle

    self.addCommands(shoot_cycle(container, indent=1))                     # the common case
    self.addCommands(shoot_cycle(container, timeout=5, intake='none'))     # short shot, no intake stage
"""

import commands2
from commands2 import SequentialCommandGroup, WaitCommand, InstantCommand

from constants import AutoConstants as ac
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.rotate_to_target import RotateToTarget
from helpers import joysticks as js

# How the intake behaves while we shoot.
#   'two_stage' - deploy to 'shoot', then to 'shoot2' after another delay (the usual case)
#   'one_stage' - deploy to 'shoot' only
#   'none'      - leave the intake alone
k_intake_modes = ('two_stage', 'one_stage', 'none')

# Which command owns the swerve and points the nose at the hub while the shooter runs.
#   'joystick' - DriveByJoystickSubsystemTargeting.  What the autos have always used.  It aims
#                correctly, but it ALSO reads the driver's sticks, and the FMS sends joystick
#                data during auto.  Kept as the default so this refactor changes nothing today.
#   'rotate'   - RotateToTarget.  Same aiming, cannot read a joystick.  Flip this one word when
#                you have had a chance to test it, and all 22 call sites change with it.
k_default_aim = 'joystick'


def _intake_sequence(container, mode: str, timeout: float, indent: int) -> commands2.Command:
    """The 'get the intake out of the ball path and then squeeze' stage."""
    steps = [
        WaitCommand(ac.k_intake_raise_delay),
        Intake_Deploy(intake=container.intake, position='shoot', indent=indent),
        Intake_Set_RPM(intake=container.intake, rpm=500),
    ]
    if mode == 'two_stage':
        steps += [
            WaitCommand(ac.k_intake_raise_delay),
            Intake_Deploy(intake=container.intake, position='shoot2', indent=indent),
        ]
    steps.append(WaitCommand(5))  # long enough that the ShootingCommand timeout is what ends the race
    return SequentialCommandGroup(*steps).withTimeout(timeout)


def shoot_cycle(container, timeout: float = ac.k_shooting_timeout, delay_cycles: int = 10,
                intake: str = 'two_stage', aim: str = k_default_aim, indent: int = 1) -> commands2.Command:
    """
    Track the hub, empty the hopper, stop tracking.  Returns one command you can addCommands().

    timeout      seconds before the shooter gives up (was 4.0 / 5 / 3.5 across the old copies)
    delay_cycles cycles to spin up before the indexer starts (10 in auto, 50 is the teleop default)
    intake       one of k_intake_modes
    aim          'joystick' or 'rotate' - see k_default_aim
    """
    if intake not in k_intake_modes:
        raise ValueError(f'intake must be one of {k_intake_modes}, got {intake!r}')

    if aim == 'rotate':
        aim_command = RotateToTarget(container, swerve=container.swerve,
                                     targeting=container.targeting, indent=indent)
    elif aim == 'joystick':
        # button_box deliberately NOT passed: 21 of the 22 old copies left it None, which pins
        # the afterburner off.  Only autonomous_shooting passed it, almost certainly by accident.
        aim_command = DriveByJoystickSubsystemTargeting(
            container, swerve=container.swerve, controller=js.driver_controller,
            targeting=container.targeting)
    else:
        raise ValueError(f"aim must be 'joystick' or 'rotate', got {aim!r}")

    racers = [
        ShootingCommand(shooter=container.shooter, targeting=container.targeting,
                        indent=indent, auto_timeout=timeout, delay_cycles=delay_cycles),
        aim_command,
    ]
    if intake != 'none':
        racers.append(_intake_sequence(container, intake, timeout, indent))

    return SequentialCommandGroup(
        InstantCommand(lambda: container.targeting.start_tracking()),
        commands2.ParallelRaceGroup(*racers),
        InstantCommand(lambda: container.targeting.stop_tracking()),
    )
