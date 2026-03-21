from math import radians
import math
import commands2
import typing
import ntcore
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
from wpimath.filter import SlewRateLimiter

import constants
from constants import AutoConstants as cac
from subsystems.swerve_constants import DriveConstants as dc, AutoConstantsSwerve as ac, TargetingConstants as tc
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.log_command import log_command

@log_command(console=True, nt=False, print_init=True, print_end=True)
class DriveToPoseCustomControl(commands2.Command):
    """
    A command to drive the robot to a specific pose using custom PID controllers, 
    featuring overshoot detection, slew rate limits, and stiction breaking.
    """

    def __init__(self, container, swerve: Swerve, target_pose_supplier: typing.Callable[[], typing.Optional[Pose2d]], 
                 indent=0, tolerance_type='exact') -> None:
        super().__init__()
        self.setName('DriveToPoseCustomControl')
        
        # --- Dependencies ---
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.addRequirements(self.swerve)

        # --- Configuration ---
        self.target_pose_supplier = target_pose_supplier
        self.tolerance_type = tolerance_type
        self.print_debug = True
        
        # --- State ---
        self.counter = 0
        self.tolerance_counter = 0
        self.rotation_achieved = False
        self.translation_achieved = False
        self.abort = False

        max_units_per_second = dc.kAutoSlewRate  
        self.x_limiter = SlewRateLimiter(max_units_per_second)
        self.y_limiter = SlewRateLimiter(max_units_per_second)
        self.rot_limiter = SlewRateLimiter(max_units_per_second)

        self.target_pose = Pose2d(0, 0, 0)

        # check for overshoot
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 999
        self.last_diff_y = 999
        self.last_diff_radians = 1

        self.create_controllers()
        self._init_networktables()

    def create_controllers(self):
        self.x_pid = PIDController(tc.kAutoTranslationPID.kP, tc.kAutoTranslationPID.kI, tc.kAutoTranslationPID.kD)
        self.x_pid.setIntegratorRange(-0.1, 0.1)  
        self.x_pid.setIZone(0.25)  

        self.y_pid = PIDController(tc.kAutoTranslationPID.kP, tc.kAutoTranslationPID.kI, tc.kAutoTranslationPID.kD)
        self.y_pid.setIntegratorRange(-0.1, 0.1)  
        self.y_pid.setIZone(0.25)  

        self.rot_pid = PIDController(tc.kAutoRotationPID.kP, tc.kAutoRotationPID.kI, tc.kAutoRotationPID.kD)
        self.rot_pid.enableContinuousInput(radians(-180), radians(180))

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        prefix = constants.auto_prefix

        self.x_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/x_setpoint").publish()
        self.y_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/y_setpoint").publish()
        self.rot_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/rot_setpoint").publish()
        self.x_measured_pub = self.inst.getDoubleTopic(f"{prefix}/x_measured").publish()
        self.y_measured_pub = self.inst.getDoubleTopic(f"{prefix}/y_measured").publish()
        self.rot_measured_pub = self.inst.getDoubleTopic(f"{prefix}/rot_measured").publish()
        self.x_commanded_pub = self.inst.getDoubleTopic(f"{prefix}/x_commanded").publish()
        self.y_commanded_pub = self.inst.getDoubleTopic(f"{prefix}/y_commanded").publish()
        self.rot_commanded_pub = self.inst.getDoubleTopic(f"{prefix}/rot_commanded").publish()

        self.auto_active_pub = self.inst.getBooleanTopic(f"{prefix}/robot_in_auto").publish()
        self.goal_pose_pub = self.inst.getStructTopic(f"{prefix}/goal_pose", Pose2d).publish()
        self.auto_active_pub.set(False)

    def reset_controllers(self):
        target = self.target_pose_supplier()
        if target is None:
            self.abort = True
            self.target_pose = self.container.swerve.get_pose()
        else:
            self.target_pose = target

        self.x_pid.setSetpoint(self.target_pose.X())
        self.y_pid.setSetpoint(self.target_pose.Y())
        self.rot_pid.setSetpoint(self.target_pose.rotation().radians())

        # reset overshoot
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 99  
        self.last_diff_y = 99
        self.last_diff_radians = 9
        self.tolerance_counter = 0
        self.rotation_achieved = False
        self.translation_achieved = False
        self.rot_limiter.reset(0)
        self.x_limiter.reset(0)
        self.y_limiter.reset(0)
        self.counter = 0

        self.x_pid.reset()
        self.y_pid.reset()
        self.rot_pid.reset()

        self.x_commanded_pub.set(self.target_pose.X())
        self.y_commanded_pub.set(self.target_pose.Y())
        self.rot_commanded_pub.set(self.target_pose.rotation().degrees())

        self.goal_pose_pub.set(self.target_pose)

    def initialize(self) -> None:
        self.abort = False  
        self.reset_controllers() 
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        self.auto_active_pub.set(True)
        self.extra_log_info = f'target {self.target_pose}'

    def execute(self) -> None:
        if self.counter == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
            print(f'CNT  DX     XT?   Xo   |   DY    YT?   Yo   |   DR     RT?   Ro   | TC')

        robot_pose = self.swerve.get_pose()

        x_output = self.x_pid.calculate(robot_pose.X())
        y_output = self.y_pid.calculate(robot_pose.Y())
        rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())

        error_vector = self.target_pose.translation() - robot_pose.translation()
        diff_x = error_vector.X()
        diff_y = error_vector.Y()
        diff_radians = (self.target_pose.rotation() - robot_pose.rotation()).radians()

        if abs(diff_x) > abs(self.last_diff_x) and self.counter > 0: self.x_overshot = True
        self.last_diff_x = diff_x
        
        if abs(diff_y) > abs(self.last_diff_y) and self.counter > 0: self.y_overshot = True
        self.last_diff_y = diff_y
        
        if abs(diff_radians) > abs(self.last_diff_radians) and self.counter > 0: self.rot_overshot = True
        self.last_diff_radians = diff_radians

        rot_max, rot_min = 0.5, 0.1
        trans_max, trans_min = 0.4, 0.1 

        if abs(x_output) < trans_min and not self.x_overshot and abs(diff_x) > ac.k_translation_tolerance_meters:
            x_output = math.copysign(trans_min, x_output)
        if abs(y_output) < trans_min and not self.y_overshot and abs(diff_y) > ac.k_translation_tolerance_meters:
            y_output = math.copysign(trans_min, y_output)
        if abs(rot_output) < rot_min and not self.rot_overshot and abs(math.degrees(diff_radians)) > ac.k_rotation_tolerance.degrees():
            rot_output = math.copysign(rot_min, rot_output)

        x_output = math.copysign(min(abs(x_output), trans_max), x_output)
        y_output = math.copysign(min(abs(y_output), trans_max), y_output)
        rot_output = math.copysign(min(abs(rot_output), rot_max), rot_output)

        x_output = self.x_limiter.calculate(x_output)
        y_output = self.y_limiter.calculate(y_output)
        rot_output = self.rot_limiter.calculate(rot_output)

        self.swerve.drive(x_output, y_output, rot_output, fieldRelative=True, rate_limited=False, keep_angle=False)

        self.rotation_achieved = abs(math.degrees(diff_radians)) < ac.k_rotation_tolerance.degrees()
        self.translation_achieved = error_vector.norm() < ac.k_translation_tolerance_meters
        if self.rotation_achieved and self.translation_achieved:
            self.tolerance_counter += 1
        else:
            self.tolerance_counter = 0

        if self.counter % 10 == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
            msg = f'{self.counter:3d}  {diff_x:+.2f} {str(self.x_overshot):>5} {x_output:+.2f} | {diff_y:+.2f}  {str(self.y_overshot):>5} {y_output:+.2f} | {math.degrees(diff_radians):>+6.1f}° {str(self.rot_overshot):>5} {rot_output:+.2f} | {self.tolerance_counter} '
            print(msg)

            if wpilib.RobotBase.isSimulation():
                self.x_setpoint_pub.set(self.x_pid.getSetpoint())
                self.y_setpoint_pub.set(self.y_pid.getSetpoint())
                self.rot_setpoint_pub.set(math.degrees(self.rot_pid.getSetpoint()))
                self.x_measured_pub.set(robot_pose.x)
                self.y_measured_pub.set(robot_pose.y)
                self.rot_measured_pub.set(robot_pose.rotation().degrees())

        self.counter += 1

    def isFinished(self) -> bool:
        if self.abort: return True
        if self.tolerance_type == 'fast':
            if self.rotation_achieved and (self.translation_achieved or self.tolerance_counter > 1 or (self.x_overshot and self.y_overshot)):
                return True
            return False
        return self.tolerance_counter > 10

    def end(self, interrupted: bool) -> None:
        if wpilib.RobotBase.isSimulation(): self.auto_active_pub.set(False)
        indicator = Led.Indicator.kFAILUREFLASH if (interrupted or self.abort) else Led.Indicator.kSUCCESSFLASH
        commands2.CommandScheduler.getInstance().schedule(self.container.led.set_indicator_with_timeout(indicator, 2))