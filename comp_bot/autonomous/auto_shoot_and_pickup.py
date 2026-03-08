import commands2

import constants
from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from commands.auto_to_pose_clean import AutoToPoseClean

from helpers import joysticks as js
from wpimath.geometry import Pose2d

class AutoShootAndPickup(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'AutoShootAndPickup')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.start_tracking()))

        self.addCommands(Intake_Deploy(intake=container.intake, position='shoot', indent=1))

        self.addCommands(commands2.WaitCommand(0.5))

        # because the drive by velocity needs swerve, we have to actively use the swerve to auto target
        self.addCommands(commands2.ParallelRaceGroup(
            ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=5),
            DriveByJoystickSubsystemTargeting(self.container, swerve=self.container.swerve, controller=js.driver_controller, targeting=container.targeting)
        ))

        # self.addCommands(ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=5))

        self.addCommands(commands2.InstantCommand(lambda: self.container.targeting.stop_tracking()))

        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=constants.IntakeConstants.k_intake_default_rpm))

        self.addCommands(commands2.ParallelCommandGroup(
            commands2.WaitCommand(1).andThen(Intake_Deploy(intake=container.intake, position='down', indent=1)),
            AutoToPoseClean(container=self.container, swerve=self.container.swerve, target_pose=None,
                            mode="ball_pickup", from_robot_state=True, control_type='not_pathplanner')
        ))

        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # flight simulator rules - y axis is reversed, so negative numbers go forward on field relative
        #self.addCommands(DriveByVelocitySwerve(self.container, self.container.swerve, Pose2d(-0.25, 0, 0), field_relative=True, indent=1, timeout=2))
        self.addCommands(AutoToPoseClean(container=self.container, swerve=self.container.swerve, target_pose=None, mode="ball_pickup", from_robot_state=True,control_type='not_pathplanner'))
        

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

