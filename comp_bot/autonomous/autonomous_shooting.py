import commands2

from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.intake_deploy import Intake_Deploy
from commands.shooting_command import ShootingCommand

from helpers import joysticks as js
from wpimath.geometry import Pose2d

class AutoShootingGroup(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'AutoShootingGroup')
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

        self.addCommands(Intake_Deploy(intake=container.intake, position='down', indent=1))

        self.addCommands(DriveByVelocitySwerve(self.container, self.container.swerve, Pose2d(0.15, 0, 0), field_relative=True, indent=1, timeout=2))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

