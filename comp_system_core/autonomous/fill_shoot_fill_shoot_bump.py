import wpilib
import commands2
from commands2 import SequentialCommandGroup, WaitCommand, ParallelCommandGroup, InstantCommand

import constants
from constants import AutoConstants as ac
from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from autonomous.shoot_cycle import shoot_cycle
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

from helpers import joysticks as js
from helpers.apriltag_utils import auto_reflect_pose
from wpimath import Pose2d

class FillShootFillShootBump(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.set_name(f'FillShootFillShootBump')
        self.container = container


        # -----  PHASE I:  DRIVE TO FILL HOPPER  -----
        # moves the intake down
        # self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # activates the intake
        # self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # moves to the neutral zone to intake fuel --> come back to shoot
        self.add_commands(
            ParallelCommandGroup(
                DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_first_ball_pickup_pose, wpilib.MatchState.get_alliance(), is_shooting=False),
                                                  tolerance_type='fast').with_timeout(5),
                SequentialCommandGroup(
                    WaitCommand(1),
                        Intake_Deploy(intake=container.intake, position='down', indent=1).and_then(
                            Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm)
                    )
                )
            )
        )

        # start the shooter on the way back so we don't waste a second letting it spin up
        self.add_commands(
            ParallelCommandGroup(
            DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                                     target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose,
                                                                                    wpilib.MatchState.get_alliance(), is_shooting=True),
                                     tolerance_type='fast').with_timeout(5),
            SequentialCommandGroup(
                WaitCommand(1), InstantCommand(lambda: self.container.shooter.set_shooter_rpm(ac.k_shooter_startup_rpm)))
        ))



        # -----  PHASE II:  SHOOT INITIAL HOPPER -----

        # Tracks the hub

        # Starts the shooting cycle and then raises the intake after a delay to prevent compression and jams
        # forces it to die when the first command finishes
        self.add_commands(shoot_cycle(self.container, indent=1))
        # stops tracking

        # -----  PHASE III:  FILL HOPPER AGAIN -----
        # Moves the intake down
        self.add_commands(Intake_Deploy(intake=container.intake, position='down', indent=1))
        self.add_commands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # Repeat what happened above
        self.add_commands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_second_ball_pickup_pose, wpilib.MatchState.get_alliance(), is_shooting=False),
                                                  tolerance_type='fast').with_timeout(4.5)
        )

        # start the shooter on the way back so we don't waste a second letting it spin up
        self.add_commands(
            ParallelCommandGroup(
            DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
                                     target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose,
                                                                                    wpilib.MatchState.get_alliance(), is_shooting=True),
                                     tolerance_type='fast').with_timeout(4),
            SequentialCommandGroup(
                WaitCommand(.5), InstantCommand(lambda: self.container.shooter.set_shooter_rpm(ac.k_shooter_startup_rpm)))
        ))


        # -----  PHASE IV:  EMPTY THE HOPPER (as above) -----
        self.add_commands(shoot_cycle(self.container, indent=1))

        self.add_commands(Intake_Set_RPM(intake=self.container.intake, rpm=0))

        self.add_commands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.get_name()} **"))
