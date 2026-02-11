
import math
import commands2
import wpilib
import ntcore

import constants
from wpimath.controller import PIDController
from subsystems.swerve import Swerve  # allows us to access the definitions
from commands2.button import CommandXboxController
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer, SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds
from subsystems.swerve_constants import DriveConstants as dc, AutoConstants as ac, TargetingConstants as tc
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
class DriveByJoystickSwerveTargeting(commands2.Command):
    def __init__(self, container, swerve: Swerve, controller: CommandXboxController, rate_limited=False) -> None:
        super().__init__()
        self.setName('drive_by_joystick_swerve_targeting')
        
        # -----------------------------------------------------------
        # 1. Subsystems & Dependencies
        # -----------------------------------------------------------
        self.container = container
        self.swerve = swerve
        self.addRequirements(self.swerve)
        self.controller: CommandXboxController = controller

        # -----------------------------------------------------------
        # 2. Configuration & State
        # -----------------------------------------------------------
        self.field_oriented = True
        self.rate_limited = rate_limited
        self.prev_commanded_vector = Translation2d(0, 0) # we should start stationary so this should be valid

        # -----------------------------------------------------------
        # 3. Input Processing (Debouncers, Limiters)
        # -----------------------------------------------------------
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        
        # CJH added a slew rate limiter 20250311 - but there already is one in Swerve, so is this redundant?
        # make sure you put it on the joystick (not calculations), otherwise it doesn't help much on slow-mode
        stick_max_units_per_second = 3  # can't be too low or you get lag - probably should be between 3 and 5
        self.drive_limiter = SlewRateLimiter(stick_max_units_per_second)
        self.strafe_limiter = SlewRateLimiter(stick_max_units_per_second)
        self.turbo_limiter = SlewRateLimiter(10)
        
        # Rotation limiters - we want a separate one for manual
        # vs tracking to allow more aggressive tracking response
        self.manual_rot_limiter = SlewRateLimiter(stick_max_units_per_second)

        # -----------------------------------------------------------
        # 4. Targeting Setup
        # -----------------------------------------------------------
        self.bHubLocation = Translation2d(4.74, 4.05)
        self.rHubLocation = Translation2d(12.1, 4.05)
        
        # PID for rotation tracking (Best parts of AutoToPoseClean)
        self.rot_pid = PIDController(tc.kTeleopRotationPID.kP, tc.kTeleopRotationPID.kI, tc.kTeleopRotationPID.kD)
        self.rot_pid.enableContinuousInput(-math.pi, math.pi)
        
        # Tracking state
        self.rot_overshot = False
        self.last_diff_radians = 100.0
        self.last_tracking_on = False
        self.counter = 0

        # debug prints for optimization        
        self.debug_prints = True
        self.debug_v_field = Translation2d()
        self.debug_future_pose = Translation2d()
        self.debug_pid = 0
        self.debug_ff = 0
        self.debug_ks = 0
        self.debug_error_deg = 0

        # -----------------------------------------------------------
        # 5. NetworkTables
        # -----------------------------------------------------------
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        status_prefix = constants.status_prefix
        # Simulation Debugging Publishers
        self.js_dv1_x_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv1_x").publish()
        self.js_dv1_y_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv1_y").publish()
        self.js_dv_norm_x_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv_norm_x").publish()
        self.js_dv_norm_y_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv_norm_y").publish()
        self.commanded_values_pub = self.inst.getDoubleArrayTopic(f"{status_prefix}/_joystick_commanded_values").publish()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        if self.debug_prints:
            print(f"CNT | Cur X  Cur Y | Cur VX Cur VY | Fut X  Fut Y | Err(°) | PID    FF     kS     Tot")

    def execute(self) -> None:
        self.counter += 1
        
        # -----------------------------------------------------------
        # 1. READ INPUTS
        # -----------------------------------------------------------
        hid = self.controller.getHID()
        inputs = {
            'robot_pose': self.swerve.get_pose(),
            'left_y': hid.getLeftY(),
            'left_x': hid.getLeftX(),
            'right_x': hid.getRightX(),
            'trigger': hid.getRightTriggerAxis(),
            'robot_oriented': hid.getLeftBumperButton(),
            'tracking_on': hid.getRightBumper(),
            'alliance': wpilib.DriverStation.getAlliance()
        }

        # -----------------------------------------------------------
        # 2. CALCULATE
        # -----------------------------------------------------------
        
        # --- Drive Mode & Multipliers ---
        turbo = self.turbo_limiter.calculate(inputs['trigger']**2)
        slowmode_multiplier = 0.2 + 0.8 * turbo
        angular_slowmode_multiplier = 0.5 + 0.5 * turbo

        # --- Field Oriented Logic ---
        if self.robot_oriented_debouncer.calculate(inputs['robot_oriented']):
            self.field_oriented = False
        else:
            self.field_oriented = True

        # --- Translation Processing ---
        # Returns (desired_fwd, desired_strafe, raw_vector)
        desired_fwd, desired_strafe, raw_vector = self._process_translation(inputs, slowmode_multiplier)

        # --- Rotation Processing: Manual vs shot-tracking ---
        raw_rot = -inputs['right_x']  # likely because we are CCW + on robot but want robot to turn CW with + stick input
        
        if inputs['tracking_on']:
            # go through the tracking calculations - allow arbitrary complexity
            desired_rot = self._calculate_tracking_rotation(inputs['robot_pose'])
            
            if self.debug_prints and self.counter % 10 == 0:
                pose = inputs['robot_pose']
                print(f"{self.counter:3d} | {pose.X():.2f} {pose.Y():.2f} | {self.debug_v_field.X():.2f} {self.debug_v_field.Y():.2f} | {self.debug_future_pose.X():.2f} {self.debug_future_pose.Y():.2f} | {self.debug_error_deg:6.1f} | {self.debug_pid:.2f} {self.debug_ff:.2f} {self.debug_ks:.2f} {desired_rot:.2f}")
        elif self.last_tracking_on: # Falling Edge
            # Reset manual limiter to current input to prevent ghost slewing
            manual_rot = raw_rot * angular_slowmode_multiplier
            self.manual_rot_limiter.reset(manual_rot)
            desired_rot = manual_rot
        else:
            desired_rot = self._calculate_manual_rotation(raw_rot, angular_slowmode_multiplier)

        self.last_tracking_on = inputs['tracking_on']

        # -----------------------------------------------------------
        # 3. ACT
        # -----------------------------------------------------------
        self.swerve.drive(
            xSpeed=desired_fwd,
            ySpeed=desired_strafe,
            rot=desired_rot,
            fieldRelative=self.field_oriented,
            rate_limited=False, # We handle all limiting in this command now
            keep_angle=False
        )

        # -----------------------------------------------------------
        # 4. REPORT
        # -----------------------------------------------------------
        if wpilib.RobotBase.isSimulation():
            # Report Raw Input
            self.js_dv1_x_pub.set(math.fabs(raw_vector.X()))
            self.js_dv1_y_pub.set(math.fabs(raw_vector.Y()))
            
            # Report Final Processed Input
            self.js_dv_norm_x_pub.set(math.fabs(desired_fwd))
            self.js_dv_norm_y_pub.set(math.fabs(desired_strafe))
            self.commanded_values_pub.set([desired_fwd, desired_strafe, desired_rot])

    def _process_translation(self, inputs, multiplier):
        # Apply calibration offsets and negation
        joystick_fwd = -(inputs['left_y'] - self.swerve.thrust_calibration_offset)
        joystick_strafe = -(inputs['left_x'] - self.swerve.strafe_calibration_offset)
        
        raw_vector = Translation2d(joystick_fwd, joystick_strafe)
        processing_vector = raw_vector

        # Deadband & Clipping
        if processing_vector.norm() > dc.k_outer_deadband:
            processing_vector *= 1 / processing_vector.norm()
        elif processing_vector.norm() < dc.k_inner_deadband:
            processing_vector = Translation2d(0, 0)

        # Response Curve (Sqrt)
        processing_vector *= math.sqrt(processing_vector.norm())
        
        # Scaling
        processing_vector *= multiplier

        # Rate Limiting
        desired_fwd = self.drive_limiter.calculate(processing_vector.X())
        desired_strafe = self.strafe_limiter.calculate(processing_vector.Y())

        # Alliance Adjustment
        if inputs['alliance'] == wpilib.DriverStation.Alliance.kRed and self.field_oriented:
            desired_fwd *= -1
            desired_strafe *= -1
            
        return desired_fwd, desired_strafe, raw_vector

    def _calculate_manual_rotation(self, raw_rot, multiplier):
        if abs(raw_rot) < dc.k_inner_deadband:
            raw_rot = 0
        
        desired_rot = raw_rot * multiplier
        return self.manual_rot_limiter.calculate(desired_rot)

    def _calculate_tracking_rotation(self, robot_pose):
        # --- Targeting Calculations ---
        # Calculate robot velocity vector
        robot_vel = self.swerve.get_relative_speeds() # Robot relative ChassisSpeeds
        v_robot = Translation2d(robot_vel.vx, robot_vel.vy)
        v_field = v_robot.rotateBy(robot_pose.rotation()) # Convert to field relative

        # Predict future position based on lookahead time  TODO - pop up a ghost like in autotoposeclean
        future_robot_location = robot_pose.translation() + (v_field * tc.k_targeting_lookahead_s)
        self.debug_v_field = v_field
        self.debug_future_pose = future_robot_location

        # Determine target based on future position  
        if (abs(self.rHubLocation - future_robot_location) < abs(self.bHubLocation - future_robot_location)):
            target_location = self.rHubLocation
        else:
            target_location = self.bHubLocation
        
        # Calculate setpoint angle based on future position (Lookahead)
        robot_to_hub_angle = (target_location - future_robot_location).angle()

        # Calculate vector from current position for Feedforward
        current_hub_vector = target_location - robot_pose.translation()
        
        # Reset state on rising edge
        if not self.last_tracking_on:
            self.rot_pid.reset()
            self.rot_overshot = False
            self.last_diff_radians = 100.0

        # Calculate PID based on where we will be in tc.k_targeting_lookahead_s seconds
        pid_output = self.rot_pid.calculate(robot_pose.rotation().radians(), robot_to_hub_angle.radians())
        self.debug_pid = pid_output
        rot_output = pid_output
        
        # --- Feedforward (Velocity Lead) ---
        # Calculate angular velocity required to track target while moving: omega = (v_field x r_target) / |r|^2
        dist_sq = current_hub_vector.norm()**2
        ff_output = 0
        if dist_sq > 0.05: # Avoid division by zero (approx 22cm)
            # 2D Cross product: vx*ry - vy*rx gives tangential velocity * r
            omega_rad_s = (v_field.X() * current_hub_vector.Y() - v_field.Y() * current_hub_vector.X()) / dist_sq
            
            # Warning if we are physically limited
            if abs(omega_rad_s) > dc.kMaxAngularSpeed and self.counter % 10 == 0:
                print(f"WARNING: Targeting limited! Req: {abs(omega_rad_s):.1f} rad/s, Max: {dc.kMaxAngularSpeed:.1f}")

            # Add Feedforward (normalized) to PID output
            ff_output = (omega_rad_s / dc.kMaxAngularSpeed) * tc.k_teleop_rotation_kf

        self.debug_ff = ff_output
        rot_output += ff_output

        # Error analysis for fine-tuning
        diff_radians = self.rot_pid.getPositionError()
        self.debug_error_deg = math.degrees(diff_radians)
        if abs(diff_radians) > abs(self.last_diff_radians):
            self.rot_overshot = True
        self.last_diff_radians = diff_radians
        
        # Apply kS (Static Friction) if we are not at the target
        ks_output = 0
        if abs(diff_radians) > tc.k_rotation_tolerance.radians():
            ks_output = math.copysign(tc.k_teleop_rotation_kS, rot_output)
            
        self.debug_ks = ks_output
        rot_output += ks_output
            
        # Clamp maximum
        rot_max = 1.0
        rot_output = math.copysign(min(abs(rot_output), rot_max), rot_output)
        
        return rot_output

    def end(self, interrupted: bool) -> None:
        # probably should leave the wheels where they are?
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)
