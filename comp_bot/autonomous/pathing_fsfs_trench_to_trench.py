import wpilib

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.path import PathPlannerPath

import commands2
from commands2 import SequentialCommandGroup, WaitCommand, ParallelCommandGroup, InstantCommand, ConditionalCommand

import constants
from constants import AutoConstants as ac
from constants import FieldConstants as fc

from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from commands.drive_to_pose_custom_control import DriveToPoseCustomControl

from helpers import joysticks as js
from helpers.apriltag_utils import auto_reflect_pose
from wpimath.geometry import Pose2d

class PathingFSFSTrenchtoTrench(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'PathingFSFSTrench-to-Trench')
        self.container = container


        # -----  PHASE I:  DRIVE TO FILL HOPPER  -----
        #self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # activates the intake
        #self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # moves to the neutral zone to intake fuel --> come back to shoot
        self.addCommands(
            ConditionalCommand(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Right_Trench_to_Trench')),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Left_Trench_to_Trench')),
                self.get_is_right
            )

        )

        # -----  PHASE II:  SHOOT INITIAL HOPPER -----
        # Tracks the hub
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.start_tracking()))

        # Starts the shooting cycle and then raises the intake after a delay to prevent compression and jams
        # forces it to die when the first command finishes
        
        self.addCommands(commands2.ParallelRaceGroup(
            ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=ac.k_shooting_timeout, delay_cycles=10),
            DriveByJoystickSubsystemTargeting(self.container, swerve=self.container.swerve, controller=js.driver_controller, targeting=container.targeting),
            SequentialCommandGroup(
                WaitCommand(ac.k_intake_raise_delay),
                Intake_Deploy(intake=self.container.intake, position='shoot', indent=1),
                Intake_Set_RPM(intake=self.container.intake, rpm=500),
                WaitCommand(5)
            ).withTimeout(ac.k_shooting_timeout)
        ))
        # stops tracking
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.stop_tracking()))

        # -----  PHASE III:  FILL HOPPER AGAIN -----
        # Moves the intake down
        # self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))
        # self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=ac.k_intake_roller_rpm))

        # # Repeat what happened above
        # self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
        #                     target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_second_ball_pickup_pose, wpilib.DriverStation.getAlliance(), is_shooting=False),
        #                                           tolerance_type='fast').withTimeout(4.5)
        # )

        # # start the shooter on the way back so we don't waste a second letting it spin up
        # self.addCommands(
        #     ParallelCommandGroup(
        #     DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
        #                              target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose,
        #                                                                             wpilib.DriverStation.getAlliance(), is_shooting=True),
        #                              tolerance_type='fast').withTimeout(5),
        #     SequentialCommandGroup(
        #         WaitCommand(1), InstantCommand(lambda: self.container.shooter.set_shooter_rpm(ac.k_shooter_startup_rpm)))
        # ))

        self.addCommands(
            ConditionalCommand(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Right_Trench_to_Trench_Refill')),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('Left_Trench_to_Trench_Refill')),
                self.get_is_right
            )

        )


        # -----  PHASE IV:  EMPTY THE HOPPER (as above) -----
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.start_tracking()))
        self.addCommands(commands2.ParallelRaceGroup(
            ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1,
                            auto_timeout=ac.k_shooting_timeout, delay_cycles=10),
            DriveByJoystickSubsystemTargeting(self.container, swerve=self.container.swerve,
                                              controller=js.driver_controller, targeting=container.targeting),
            SequentialCommandGroup(
                WaitCommand(ac.k_intake_raise_delay),
                Intake_Deploy(intake=self.container.intake, position='shoot', indent=1),
                Intake_Set_RPM(intake=self.container.intake, rpm=500),
                WaitCommand(5)
            ).withTimeout(ac.k_shooting_timeout)
        ))
        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.stop_tracking()))

        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=0))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

    def get_is_right(self):
        alliance_color = wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue
        is_left = self.container.swerve.get_pose().Y() > fc.k_field_width / 2
        return alliance_color ^ is_left