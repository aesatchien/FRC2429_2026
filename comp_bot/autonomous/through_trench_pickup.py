import wpilib
import commands2

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
from wpimath.geometry import Pose2d

class ThroughTrenchFillShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'ThroughTrenchFillShoot')
        self.container = container
        # PUTS THE INTAKE DOWN
        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # self.addCommands(commands2.WaitCommand(0.5))

        # because the drive by velocity needs swerve, we have to actively use the swerve to auto target
        # self.addCommands(shoot_cycle(self.container, timeout=3.5, delay_cycles=50, intake='none', indent=1))

        # self.addCommands(ShootingCommand(shooter=container.shooter, targeting=container.targeting, indent=1, auto_timeout=5))

        # ACTIVATES THE INTAKE
        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=constants.IntakeConstants.k_intake_default_rpm))

        # self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        # flight simulator rules - y axis is reversed, so negative numbers go forward on field relative
        #self.addCommands(DriveByVelocitySwerve(self.container, self.container.swerve, Pose2d(-0.25, 0, 0), field_relative=True, indent=1, timeout=2))

        # moves to the neutral zone to intake fuel --> come back to shoot
        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_first_ball_pickup_pose, wpilib.DriverStation.getAlliance(), is_shooting=False), tolerance_type='fast').withTimeout(5)
        )

        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose, wpilib.DriverStation.getAlliance(), is_shooting=True), tolerance_type='exact').withTimeout(5)
        )

        # Raises the intake to shooting position
        self.addCommands(Intake_Deploy(intake=self.container.intake, position='shoot', indent=1))
        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=0))

        # Tracks the hub

        #Shoots fuel and stops tracking
        self.addCommands(shoot_cycle(self.container, timeout=5, delay_cycles=50, intake='none', indent=1))

        # Moves the intake down
        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=constants.IntakeConstants.k_intake_default_rpm))

        # Repeat what happened above
        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_second_ball_pickup_pose, wpilib.DriverStation.getAlliance(), is_shooting=False), tolerance_type='fast').withTimeout(4.5)
        )

        self.addCommands(DriveToPoseCustomControl(container=self.container, swerve=self.container.swerve,
            target_pose_supplier=lambda: auto_reflect_pose(self.container.swerve.get_pose(), ac.k_shooting_pose, wpilib.DriverStation.getAlliance(), is_shooting=True), tolerance_type='exact').withTimeout(4.5)
        )
        self.addCommands(Intake_Set_RPM(intake=self.container.intake, rpm=0))
        self.addCommands(Intake_Deploy(intake=self.container.intake, position='shoot', indent=1))


        self.addCommands(shoot_cycle(self.container, timeout=5, delay_cycles=50, intake='none', indent=1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))
