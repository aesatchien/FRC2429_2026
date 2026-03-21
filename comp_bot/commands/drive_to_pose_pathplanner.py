from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
import typing
import ntcore
import wpilib
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d

import constants
from constants import AutoConstants as cac
from subsystems.swerve_constants import DriveConstants as dc, AutoConstantsSwerve as ac, TargetingConstants as tc
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.log_command import log_command

@log_command(console=True, nt=False, print_init=True, print_end=True)
class DriveToPosePathPlanner(commands2.Command):
    """
    A command to drive the robot to a specific pose using PathPlanner's holonomic controller.
    """

    def __init__(self, container, swerve: Swerve, target_pose_supplier: typing.Callable[[], typing.Optional[Pose2d]], 
                 indent=0, tolerance_type='exact') -> None:
        super().__init__()
        self.setName('DriveToPosePathPlanner')
        
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

        self.target_pose = Pose2d(0, 0, 0)

        # check for overshoot
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 999
        self.last_diff_y = 999
        self.last_diff_radians = 1
        self.error_distance = 999.0
        self.error_degrees = 999.0

        self.target_state = PathPlannerTrajectoryState()
        self.target_state.feedforwards = DriveFeedforwards()
        self.target_state_flipped = self.target_state

        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        prefix = constants.auto_prefix

        self.x_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/x_setpoint").publish()
        self.y_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/y_setpoint").publish()
        self.rot_setpoint_pub = self.inst.getDoubleTopic(f"{prefix}/rot_setpoint").publish()
        self.x_measured_pub = self.inst.getDoubleTopic(f"{prefix}/x_measured").publish()
        self.y_measured_pub = self.inst.getDoubleTopic(f"{prefix}/y_measured").publish()
        self.rot_measured_pub = self.inst.getDoubleTopic(f"{prefix}/rot_measured").publish()

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
        
        self.target_state.pose = self.target_pose
        self.target_state.heading = self.target_pose.rotation()

        if self.swerve.flip_path():
            self.target_state_flipped = self.target_state.flip()
        else:
            self.target_state_flipped = self.target_state

        # reset overshoot
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 99  
        self.last_diff_y = 99
        self.last_diff_radians = 9
        self.error_distance = 999.0
        self.error_degrees = 999.0
        self.tolerance_counter = 0
        self.rotation_achieved = False
        self.translation_achieved = False
        self.counter = 0

        self.goal_pose_pub.set(self.target_pose)

    def initialize(self) -> None:
        self.abort = False
        self.reset_controllers()
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        self.auto_active_pub.set(True)
        self.extra_log_info = f'target {self.target_pose}'

    def execute(self) -> None:
        robot_pose = self.swerve.get_pose()

        target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(
            current_pose=robot_pose, target_state=self.target_state_flipped)
        self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

        error_vector = self.target_pose.translation() - robot_pose.translation()
        diff_radians = (self.target_pose.rotation() - robot_pose.rotation()).radians()
        
        self.error_distance = error_vector.norm()
        self.error_degrees = abs(math.degrees(diff_radians))

        self.rotation_achieved = abs(math.degrees(diff_radians)) < ac.k_rotation_tolerance.degrees()
        self.translation_achieved = error_vector.norm() < ac.k_translation_tolerance_meters
        if self.rotation_achieved and self.translation_achieved:
            self.tolerance_counter += 1
        else:
            self.tolerance_counter = 0

        self.counter += 1

    def isFinished(self) -> bool:
        if self.abort: return True
        # condition where we want to be fast - sloppy is ok
        if self.tolerance_type == 'fast':
            return self.error_distance < 0.10 and self.error_degrees < 10.0
        # default condition where we want to be exact
        return self.tolerance_counter > 10

    def end(self, interrupted: bool) -> None:
        if wpilib.RobotBase.isSimulation(): self.auto_active_pub.set(False)
        indicator = Led.Indicator.kFAILUREFLASH if (interrupted or self.abort) else Led.Indicator.kSUCCESSFLASH
        commands2.CommandScheduler.getInstance().schedule(self.container.led.set_indicator_with_timeout(indicator, 2))