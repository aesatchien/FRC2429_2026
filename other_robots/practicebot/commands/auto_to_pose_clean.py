from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
import ntcore
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
from wpimath.filter import SlewRateLimiter

import constants
from subsystems.swerve_constants import AutoConstants as ac, TargetingConstants as tc
from subsystems.swerve import Swerve
from subsystems.led import Led
from subsystems.vision import Vision
from helpers.log_command import log_command
from helpers.apriltag_utils import get_nearest_tag


@log_command(console=True, nt=False, print_init=True, print_end=True)
class AutoToPoseClean(commands2.Command):  #
    """
    A command to drive the robot to a specific pose using either PathPlanner or custom PID controllers.

    Modes of Operation (determined by arguments):
    1. Static Target: Pass `target_pose`.
    2. Vision Target: Pass `use_vision=True`. Uses cameras to find a target relative to the robot.
    3. Nearest Tag: Pass `nearest=True`. Finds nearest AprilTag and drives to a scoring location associated with it.
    4. Robot State: Pass `from_robot_state=True`. Uses a goal previously set in the RobotState subsystem.

    Control Types:
    - 'pathplanner': Uses PathPlanner's holonomic controller.
    - 'custom' (default): Uses internal PID controllers with overshoot detection and stiction handling.
    """

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, use_vision=False, cameras=None,
                 nearest=False, from_robot_state=False, control_type='not_pathplanner', indent=0,
                 offset: Transform2d = None) -> None:
        super().__init__()
        self.setName('AutoToPoseClean')  # using the pathplanner controller instead
        
        # --- Dependencies ---
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.addRequirements(self.swerve)

        # --- Configuration ---
        self.control_type = control_type
        self.from_robot_state = from_robot_state
        self.nearest = nearest  # only use nearest tags as the target
        self.use_vision = use_vision  # use the cameras to tell us where to go
        self.cameras = cameras  # which cameras to use
        self.offset = offset  # offset in robot frame (x=forward, y=left, rot=ccw)
        self.print_debug = True
        
        # --- State ---
        self.counter = 0
        self.tolerance_counter = 0
        self.abort = False

        # CJH added a slew rate limiter 20250323 - it jolts and browns out the robot if it servos to full speed
        max_units_per_second = 2  # can't be too low or you get lag and we allow a max of < 50% below
        self.x_limiter = SlewRateLimiter(max_units_per_second)
        self.y_limiter = SlewRateLimiter(max_units_per_second)
        self.rot_limiter = SlewRateLimiter(max_units_per_second)

        self.target_pose = target_pose
        if target_pose is None:
            self.target_pose = Pose2d(0, 0, 0)
        self.original_target_pose = self.target_pose

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
        if self.control_type == 'pathplanner':
            self.target_state = PathPlannerTrajectoryState()
            self.target_state.feedforwards = DriveFeedforwards()
            self.target_state_flipped = self.target_state

        else:  # custom
            # trying to get it to slow down but still make it to final position
            # after 1s at 1 error the integral contribution will be Ki - so 1s at 0.1 error will output 0.1 * Ki
            self.x_pid = PIDController(tc.kAutoTranslationPID.kP, tc.kAutoTranslationPID.kI, tc.kAutoTranslationPID.kD)
            self.x_pid.setIntegratorRange(-0.1, 0.1)  # clamp Ki * Tot Error min(negative) and max output
            self.x_pid.setIZone(0.25)  # do not allow integral unless we are within 0.25m (prevents windup)

            self.y_pid = PIDController(tc.kAutoTranslationPID.kP, tc.kAutoTranslationPID.kI, tc.kAutoTranslationPID.kD)
            self.y_pid.setIntegratorRange(-0.1, 0.1)  # clamp min(negative) and max output of the integral term
            self.y_pid.setIZone(0.25)  # do not allow integral unless we are within 0.25m (prevents windup)

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
        """Determines the target pose and resets PID/PathPlanner state."""
        self.target_pose = self._determine_target_pose()
        
        if self.control_type == 'pathplanner':
            # Update PathPlanner state
            self.target_state.pose = self.target_pose  # set the pose of the target state
            self.target_state.heading = self.target_pose.rotation()

            if self.swerve.flip_path():  # this is in initialize, not __init__, in case FMS hasn't told us the right alliance on boot-up
                self.target_state_flipped = self.target_state.flip()
            else:
                self.target_state_flipped = self.target_state

        else:  # custom
            # Update PID Setpoints
            self.x_pid.setSetpoint(self.target_pose.X())
            self.y_pid.setSetpoint(self.target_pose.Y())
            self.rot_pid.setSetpoint(self.target_pose.rotation().radians())

            # reset overshoot
            self.x_overshot = False
            self.y_overshot = False
            self.rot_overshot = False
            self.last_diff_x = 99  # has to start out a large number
            self.last_diff_y = 99
            self.last_diff_radians = 9
            self.tolerance_counter = 0
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

            # Update the goal pose for the ghost robot
            self.goal_pose_pub.set(self.target_pose)

    def _determine_target_pose(self) -> Pose2d:
        """Calculates the target pose based on the selected mode."""
        current_pose = self.container.swerve.get_pose()

        if self.nearest:
            # 1. Nearest Tag Mode
            nearest_tag = get_nearest_tag(current_pose=current_pose, destination='reef')
            self.container.robot_state.set_reef_goal_by_tag(nearest_tag)
            target = self.container.robot_state.reef_goal_pose
            # Mirroring handled below

        elif self.from_robot_state:
            # 2. Robot State Mode
            target = self.container.robot_state.reef_goal_pose

        elif self.use_vision:
            # 3. Vision Mode
            # Vision is relative to the robot, so we return immediately (no mirroring needed)
            relative_pose = self.container.vision.nearest_to_robot(self.cameras)

            if relative_pose is not None:
                # Apply offset in robot frame if provided
                if self.offset is not None:
                    rx = relative_pose.X() + self.offset.X()
                    ry = relative_pose.Y() + self.offset.Y()
                    rrot = relative_pose.rotation() + self.offset.rotation()
                    relative_pose = Pose2d(rx, ry, rrot)

                rel_tf = Transform2d(relative_pose.translation(), relative_pose.rotation())
                target = current_pose.transformBy(rel_tf)
                print(f"AutoToPose Vision: Robot Rel Move -> X: {relative_pose.X():.2f}m, Y: {relative_pose.Y():.2f}m, Rot: {relative_pose.rotation().degrees():.1f}°")
                return target
            else:
                self.abort = True
                print("AutoToPoseClean: Vision target not found. Aborting.")
                return current_pose

        else:
            # 4. Static Target Mode
            target = self.original_target_pose

        # Apply Red Alliance Mirroring to everything EXCEPT Vision
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            mid_x = constants.FieldConstants.k_field_length / 2
            mid_y = constants.FieldConstants.k_field_width / 2
            target = target.rotateAround(point=Translation2d(mid_x, mid_y), rot=Rotation2d(math.pi))
            
        return target


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.abort = False  # reset_controllers will change this if something is wrong
        self.reset_controllers()  # this is supposed to get us a new pose and reset all the parameters

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)

        self.auto_active_pub.set(True)

        self.extra_log_info = f'target {self.target_pose}'

    def execute(self) -> None:
        # moved here because of the auto-logger
        if self.counter == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
            msg = f'CNT  DX     XT?   Xo   |   DY    YT?   Yo   |   DR     RT?   Ro   | TC'
            print(msg)

        robot_pose = self.swerve.get_pose()

        if self.control_type == 'pathplanner':
            self._execute_pathplanner_control(robot_pose)
        else:
            self._execute_custom_control(robot_pose)

        self.counter += 1

    def _execute_pathplanner_control(self, robot_pose):
        target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(
            current_pose=robot_pose, target_state=self.target_state_flipped)
        self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

    def _execute_custom_control(self, robot_pose):
        # 1. Calculate PID Outputs
        x_output = self.x_pid.calculate(robot_pose.X())
        y_output = self.y_pid.calculate(robot_pose.Y())
        rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())

        # 2. Calculate Errors (for overshoot detection and tolerance)
        error_vector = self.target_pose.translation() - robot_pose.translation()
        diff_x = error_vector.X()
        diff_y = error_vector.Y()
        diff_radians = (self.target_pose.rotation() - robot_pose.rotation()).radians()

        # 3. Check for Overshoot
        if abs(diff_x) > abs(self.last_diff_x) and self.counter > 0: self.x_overshot = True
        self.last_diff_x = diff_x
        
        if abs(diff_y) > abs(self.last_diff_y) and self.counter > 0: self.y_overshot = True
        self.last_diff_y = diff_y
        
        if abs(diff_radians) > abs(self.last_diff_radians) and self.counter > 0: self.rot_overshot = True
        self.last_diff_radians = diff_radians

        # 4. Apply Stiction Breaking (Minimum Output)
        # If we haven't overshot yet, but we are close, ensure we apply enough power to move
        rot_max, rot_min = 0.5, 0.1
        trans_max, trans_min = 0.3, 0.1

        if abs(x_output) < trans_min and not self.x_overshot and abs(diff_x) > ac.k_translation_tolerance_meters:
            x_output = math.copysign(trans_min, x_output)
        if abs(y_output) < trans_min and not self.y_overshot and abs(diff_y) > ac.k_translation_tolerance_meters:
            y_output = math.copysign(trans_min, y_output)
        if abs(rot_output) < rot_min and not self.rot_overshot and abs(math.degrees(diff_radians)) > ac.k_rotation_tolerance.degrees():
            rot_output = math.copysign(rot_min, rot_output)

        # 5. Clamp Outputs
        x_output = math.copysign(min(abs(x_output), trans_max), x_output)
        y_output = math.copysign(min(abs(y_output), trans_max), y_output)
        rot_output = math.copysign(min(abs(rot_output), rot_max), rot_output)

        # 6. Slew Rate Limiting (Smooth Acceleration)
        x_output = self.x_limiter.calculate(x_output)
        y_output = self.y_limiter.calculate(y_output)
        rot_output = self.rot_limiter.calculate(rot_output)

        # 7. Drive
        self.swerve.drive(x_output, y_output, rot_output, fieldRelative=True, rate_limited=False, keep_angle=False)

        # 8. Check Tolerance
        rotation_achieved = abs(math.degrees(diff_radians)) < ac.k_rotation_tolerance.degrees()
        translation_achieved = error_vector.norm() < ac.k_translation_tolerance_meters
        if rotation_achieved and translation_achieved:
            self.tolerance_counter += 1
        else:
            self.tolerance_counter = 0

        # 9. Logging
        if self.counter % 10 == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
            msg = f'{self.counter:3d}  {diff_x:+.2f} {str(self.x_overshot):>5} {x_output:+.2f} | {diff_y:+.2f}  {str(self.y_overshot):>5} {y_output:+.2f} '
            msg += f'| {math.degrees(diff_radians):>+6.1f}° {str(self.rot_overshot):>5} {rot_output:+.2f} | {self.tolerance_counter} '
            print(msg)

            if wpilib.RobotBase.isSimulation():
                self.x_setpoint_pub.set(self.x_pid.getSetpoint())
                self.y_setpoint_pub.set(self.y_pid.getSetpoint())
                self.rot_setpoint_pub.set(math.degrees(self.rot_pid.getSetpoint()))
                self.x_measured_pub.set(robot_pose.x)
                self.y_measured_pub.set(robot_pose.y)
                self.rot_measured_pub.set(robot_pose.rotation().degrees())

    def isFinished(self) -> bool:
        return self.tolerance_counter > 10 or self.abort

    def end(self, interrupted: bool) -> None:
        if wpilib.RobotBase.isSimulation():  # Cancel the Ghost Robot
            self.auto_active_pub.set(False)

        if interrupted or self.abort:  # we killed ourself, so we want this case to also to be red
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kFAILUREFLASH, 2))
        else:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))