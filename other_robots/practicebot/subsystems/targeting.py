import math
import ntcore
import wpilib
from commands2 import Subsystem
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

import constants
from subsystems.swerve_constants import DriveConstants as dc, TargetingConstants as tc

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

        # Debugging variables
        self.debug_v_field = Translation2d()
        self.debug_future_pose = Translation2d()
        self.debug_pid = 0
        self.debug_ff = 0
        self.debug_ks = 0
        self.debug_error_deg = 0

        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        status_prefix = constants.status_prefix
        self.targeting_debug_pub = self.inst.getDoubleArrayTopic(f"{status_prefix}/targeting_debug").publish()

    def reset_state(self):
        """Resets the PID controller and tracking state. Call this when tracking starts."""
        self.rot_pid.reset()
        self.rot_overshot = False
        self.last_diff_radians = 100.0

    def calculate_target_rotation(self, robot_pose: Pose2d, robot_vel: ChassisSpeeds) -> float:
        """
        Calculates the required rotation speed to track the hub.
        Includes Lookahead, Physics Feedforward, PID, and Static Friction compensation.
        """
        self.counter += 1
        self.last_robot_pose = robot_pose

        # --- Targeting Calculations ---
        
        # 1. Calculate robot velocity vector (Field Relative)
        v_robot = Translation2d(robot_vel.vx, robot_vel.vy)
        v_field = v_robot.rotateBy(robot_pose.rotation()) 

        # 2. Predict future position based on lookahead time (Lag Compensation)
        future_robot_location = robot_pose.translation() + (v_field * tc.k_targeting_lookahead_s)
        self.debug_v_field = v_field
        self.debug_future_pose = future_robot_location

        # 3. Determine target based on future position (prevents jitter)
        if (abs(self.rHubLocation - future_robot_location) < abs(self.bHubLocation - future_robot_location)):
            target_location = self.rHubLocation
        else:
            target_location = self.bHubLocation
        
        # 4. Calculate setpoint angle based on future position (Lookahead)
        robot_to_hub_angle = (target_location - future_robot_location).angle()

        # 5. Calculate vector from current position for Feedforward
        current_hub_vector = target_location - robot_pose.translation()

        # 6. Calculate PID Output
        # We use the Lookahead angle as the setpoint, but the current rotation as measurement
        pid_output = self.rot_pid.calculate(robot_pose.rotation().radians(), robot_to_hub_angle.radians())
        self.debug_pid = pid_output
        rot_output = pid_output
        
        # 7. Feedforward (Velocity Lead)
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
        
        # 9. Apply kS (Static Friction) if we are not at the target
        ks_output = 0
        if abs(diff_radians) > tc.k_rotation_tolerance.radians():
            ks_output = math.copysign(tc.k_teleop_rotation_kS, rot_output)
            
        self.debug_ks = ks_output
        rot_output += ks_output
            
        # 10. Clamp maximum
        rot_max = 1.0
        rot_output = math.copysign(min(abs(rot_output), rot_max), rot_output)
        
        self.last_rot_output = rot_output
        return rot_output

    def periodic(self):
        # Publish debug data periodically
        if self.counter % 10 == 0:
            self.targeting_debug_pub.set([
                self.last_robot_pose.X(), self.last_robot_pose.Y(),
                self.debug_v_field.X(), self.debug_v_field.Y(),
                self.debug_future_pose.X(), self.debug_future_pose.Y(),
                self.debug_error_deg,
                self.debug_pid, self.debug_ff, self.debug_ks, self.last_rot_output
            ])