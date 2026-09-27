from math import radians
import math
import commands2
from helpers.dashboard import SmartDashboard  # 2027a7: wpilib's was removed
from wpimath import PIDController
from wpimath import Pose2d

from subsystems.swerve_constants import AutoConstantsSwerve as ac
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class PIDToPoint(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, control_type=None, indent=0) -> None:
        """
        this command handles flipping for red alliance, so only ever pass it things which apply to blue alliance
        """
        super().__init__()
        self.set_name('PID to point')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = swerve

        self.target_pose = target_pose

        self.x_pid = PIDController(1, 0, 0.1)
        self.y_pid = PIDController(1, 0, 0.1)
        self.rot_pid = PIDController(1, 0, 0)
        self.rot_pid.enable_continuous_input(radians(-180), radians(180))
        self.x_pid.set_setpoint(target_pose.x)
        self.y_pid.set_setpoint(target_pose.y)
        self.rot_pid.set_setpoint(target_pose.rotation().radians())

        SmartDashboard.put_number("x commanded", 0)
        SmartDashboard.put_number("y commanded", 0)
        SmartDashboard.put_number("rot commanded", 0)

        self.add_requirements(self.swerve)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        self.extra_log_info = f'to {self.target_pose}'

        self.x_pid.reset()
        self.y_pid.reset()
        self.rot_pid.reset()

        self.container.led.set_indicator(Led.Indicator.kPOLKA)

    def execute(self) -> None:
        # we could also do this with wpilib pidcontrollers
        robot_pose = self.swerve.get_pose()

        x_setpoint = self.x_pid.calculate(robot_pose.x)
        y_setpoint = self.y_pid.calculate(robot_pose.y)
        rot_setpoint = self.rot_pid.calculate(robot_pose.rotation().radians())

        SmartDashboard.put_number("x setpoint", self.x_pid.get_setpoint())
        SmartDashboard.put_number("y setpoint", self.y_pid.get_setpoint())
        SmartDashboard.put_number("rot setpoint", math.degrees(self.rot_pid.get_setpoint()))

        SmartDashboard.put_number("x measured", robot_pose.x)
        SmartDashboard.put_number("y measured", robot_pose.y)
        SmartDashboard.put_number("rot measured", robot_pose.rotation().degrees())

        SmartDashboard.put_number("x commanded", x_setpoint)
        SmartDashboard.put_number("y commanded", y_setpoint)
        SmartDashboard.put_number("rot commanded", rot_setpoint)

        self.swerve.drive(x_setpoint, y_setpoint, rot_setpoint, fieldRelative=True, rate_limited=False, keep_angle=True)

    def is_finished(self) -> bool:
        diff = self.swerve.get_pose().relative_to(self.target_pose)
        rotation_achieved = abs(diff.rotation().degrees()) < ac.k_rotation_tolerance.degrees()
        translation_achieved = diff.translation().norm() < ac.k_translation_tolerance_meters
        return rotation_achieved and translation_achieved

    def end(self, interrupted: bool) -> None:
        if interrupted:
            commands2.CommandScheduler.get_instance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kFAILUREFLASH, 2))
        else:
            commands2.CommandScheduler.get_instance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))


