import math
import bisect
import ntcore
import wpilib
from commands2 import Subsystem
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
import constants
from constants import ShooterConstants as sc
from subsystems.swerve_constants import DriveConstants as dc, TargetingConstants as tc

class InterpolatedLookupTable:
    """
    Simple lookup table with linear interpolation.
    Replaces wpimath.interpolation.InterpolatingDoubleTreeMap.
    """
    def __init__(self, data: dict):
        self.x = sorted(data.keys())
        self.y = [data[k] for k in self.x]
        
    def get(self, val: float) -> float:
        if not self.x:
            return 0.0
        if val <= self.x[0]:
            return self.y[0]
        if val >= self.x[-1]:
            return self.y[-1]
            
        # Find the insertion point to maintain sorted order
        idx = bisect.bisect_right(self.x, val)
        x0, x1 = self.x[idx-1], self.x[idx]
        y0, y1 = self.y[idx-1], self.y[idx]
        
        # Linear interpolation
        return y0 + (val - x0) * (y1 - y0) / (x1 - x0)

class Targeting(Subsystem):
    """
    Functional subsystem that handles targeting calculations.
    It does not own hardware but performs the math to determine where the robot should look.
    """
    
    def __init__(self) -> None:
        super().__init__()
        self.setName('Targeting')

        # Targets (Blue and Red Hubs)
        self.bHubLocation = Translation2d(4.74, 4.05)
        self.rHubLocation = Translation2d(12.1, 4.05)

        # PID for rotation tracking
        self.rot_pid = PIDController(tc.kTeleopRotationPID.kP, tc.kTeleopRotationPID.kI, tc.kTeleopRotationPID.kD)
        self.rot_pid.enableContinuousInput(-math.pi, math.pi)

        # Tracking state
        self.rot_overshot = False
        self.last_diff_radians = 100.0
        self.counter = 0
        self.last_robot_pose = Pose2d()
        self.last_rot_output = 0
        self.effective_distance = 0.0
        self.target_rpm = 0.0
        self.shot_error = 999.0
        self.last_calc_time = -1.0
        self.was_active = False

        # Debugging variables
        self.debug_v_field = Translation2d()
        self.debug_future_pose = Translation2d()
        self.debug_future_rotation = Rotation2d()
        self.debug_shot_end = Pose2d()
        self.debug_pid = 0
        self.debug_ff = 0
        self.debug_ks = 0
        self.debug_error_deg = 0

        # Initialize Lookup Tables
        self.rpm_map = InterpolatedLookupTable(sc.k_distance_to_rpm)
        self.tof_map = InterpolatedLookupTable(sc.k_distance_to_tof)

        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        status_prefix = constants.status_prefix
        auto_prefix = constants.auto_prefix

        self.targeting_debug_pub = self.inst.getDoubleArrayTopic(f"{status_prefix}/targeting_debug").publish()
        
        # Match AutoToPoseClean publishing for Physics/Sim visualization
        self.auto_active_pub = self.inst.getBooleanTopic(f"{auto_prefix}/robot_in_auto").publish()
        self.goal_pose_pub = self.inst.getStructTopic(f"{auto_prefix}/goal_pose", Pose2d).publish()
        self.shot_line_pub = self.inst.getStructArrayTopic(f"{auto_prefix}/shot_line", Pose2d).publish()
        self.is_ok_to_shoot_pub = self.inst.getBooleanTopic(f"{auto_prefix}/is_ok_to_shoot").publish()
        self.shot_error_pub = self.inst.getDoubleTopic(f"{auto_prefix}/shot_error").publish()

    def reset_state(self):
        """Resets the PID controller and tracking state. Call this when tracking starts."""
        self.rot_pid.reset()
        self.rot_overshot = False
        self.last_diff_radians = 100.0

    def get_effective_distance(self) -> float:
        """Returns the distance from the predicted future robot pose to the target."""
        return self.effective_distance
        
    def get_target_rpm(self) -> float:
        """Returns the calculated RPM based on effective distance."""
        return self.target_rpm

    def is_at_target(self) -> bool:
        """Returns True if the rotation error is within tolerance."""
        return abs(self.debug_error_deg) < tc.k_rotation_tolerance.degrees()
        
    def is_ok_to_shoot(self) -> bool:
        """Returns True if the calculated shot endpoint is within tolerance of the target."""
        return self.shot_error < tc.kShotAccuracyToleranceMeters

    def calculate_target_rotation(self, robot_pose: Pose2d, robot_vel: ChassisSpeeds) -> float:
        """
        Calculates the required rotation speed to track the hub.
        Includes Lookahead, Physics Feedforward, PID, and Static Friction compensation.
        """
        self.counter += 1
        self.last_calc_time = wpilib.Timer.getFPGATimestamp()
        self.last_robot_pose = robot_pose
        
        # Deadband velocity to prevent jitter at start/stop
        if math.hypot(robot_vel.vx, robot_vel.vy) < 0.1:
            robot_vel = ChassisSpeeds(0, 0, 0)

        # --- Targeting Calculations ---
        
        # 1. Calculate robot velocity vector (Field Relative)
        v_robot = Translation2d(robot_vel.vx, robot_vel.vy)
        v_field = v_robot.rotateBy(robot_pose.rotation()) 

        # 2. Determine target (closest hub based on current position)
        # We do this early to calculate distance for Time of Flight
        if (abs(self.rHubLocation - robot_pose.translation()) < abs(self.bHubLocation - robot_pose.translation())):
            target_location = self.rHubLocation
        else:
            target_location = self.bHubLocation

        # 3. Calculate Dynamic Lookahead (Time of Flight)
        current_dist = robot_pose.translation().distance(target_location)
        # Look up Time of Flight based on current distance
        lookahead_time = self.tof_map.get(current_dist)

        # 4. Predict future position
        future_robot_location = robot_pose.translation() + (v_field * lookahead_time)
        self.debug_v_field = v_field
        self.debug_future_pose = future_robot_location
        
        # 5. Calculate Effective Distance (for Shooter RPM)
        self.effective_distance = future_robot_location.distance(target_location)
        self.target_rpm = self.rpm_map.get(self.effective_distance)

        # --- Calculate Shot Line for Visualization ---
        # Where the ball goes if we shoot NOW with CURRENT heading
        # Endpoint = CurrentPos + (RobotVel * t) + (ShotVel * t)
        # (ShotVel * t) is just a vector of length effective_distance in the direction of the robot
        shot_vector = Translation2d(self.effective_distance, 0).rotateBy(robot_pose.rotation())
        robot_motion_vector = v_field * lookahead_time
        
        end_point_translation = robot_pose.translation() + robot_motion_vector + shot_vector
        self.debug_shot_end = Pose2d(end_point_translation, robot_pose.rotation())
        
        # Calculate shot error (distance from shot endpoint to target center)
        self.shot_error = end_point_translation.distance(target_location)
        
        # 6. Calculate setpoint angle based on future position (Lookahead)
        robot_to_hub_angle = (target_location - future_robot_location).angle()
        self.debug_future_rotation = robot_to_hub_angle

        # 7. Calculate vector from current position for Feedforward
        current_hub_vector = target_location - robot_pose.translation()

        # 8. Calculate PID Output
        # We use the Lookahead angle as the setpoint, but the current rotation as measurement
        pid_output = self.rot_pid.calculate(robot_pose.rotation().radians(), robot_to_hub_angle.radians())
        self.debug_pid = pid_output
        rot_output = pid_output
        
        # 9. Feedforward (Velocity Lead)
        # Calculate angular velocity required to track target while moving: omega = (v_field x r_target) / |r|^2
        dist_sq = current_hub_vector.norm()**2
        ff_output = 0
        if dist_sq > 0.05: # Avoid division by zero (approx 22cm)
            # 2D Cross product: vx*ry - vy*rx gives tangential velocity * r
            omega_rad_s = (v_field.X() * current_hub_vector.Y() - v_field.Y() * current_hub_vector.X()) / dist_sq
            
            # Add Feedforward (normalized) to PID output
            ff_output = (omega_rad_s / dc.kMaxAngularSpeed) * tc.k_teleop_rotation_kf

        self.debug_ff = ff_output
        rot_output += ff_output

        # 8. Error analysis for fine-tuning (Overshoot detection)
        diff_radians = self.rot_pid.getPositionError()
        self.debug_error_deg = math.degrees(diff_radians)
        if abs(diff_radians) > abs(self.last_diff_radians):
            self.rot_overshot = True
        self.last_diff_radians = diff_radians
        
        # 10. Apply kS (Static Friction) if we are not at the target
        ks_output = 0
        if abs(diff_radians) > tc.k_rotation_tolerance.radians():
            ks_output = math.copysign(tc.k_teleop_rotation_kS, rot_output)
            
        self.debug_ks = ks_output
        rot_output += ks_output
            
        # 11. Clamp maximum
        rot_max = 1.0
        rot_output = math.copysign(min(abs(rot_output), rot_max), rot_output)
        
        self.last_rot_output = rot_output
        return rot_output

    def periodic(self):
        # Check if targeting is active (heuristic: calculated recently, e.g. < 0.1s ago)
        is_active = (wpilib.Timer.getFPGATimestamp() - self.last_calc_time) < 0.1


        if is_active:
            self.auto_active_pub.set(True)
            self.goal_pose_pub.set(Pose2d(self.debug_future_pose, self.debug_future_rotation))
            self.shot_line_pub.set([self.last_robot_pose, self.debug_shot_end])
            self.was_active = True
        elif self.was_active:
            self.auto_active_pub.set(False)
            self.shot_line_pub.set([])
            self.is_ok_to_shoot_pub.set(False)
            self.shot_error_pub.set(999.0)
            self.was_active = False

        # Publish debug data periodically
        if self.counter % 10 == 0:
            self.targeting_debug_pub.set([
                self.last_robot_pose.X(), self.last_robot_pose.Y(),
                self.debug_v_field.X(), self.debug_v_field.Y(),
                self.debug_future_pose.X(), self.debug_future_pose.Y(),
                self.debug_error_deg,
                self.debug_pid, self.debug_ff, self.debug_ks, self.last_rot_output,
                self.effective_distance, self.target_rpm,
                self.shot_error
            ])
            
            if is_active:
                self.is_ok_to_shoot_pub.set(self.is_ok_to_shoot())
                self.shot_error_pub.set(self.shot_error)
            else:
                self.is_ok_to_shoot_pub.set(False)
                self.shot_error_pub.set(999.0)