
import math
import commands2
import wpilib
import ntcore

import constants
from subsystems.swerve import Swerve  # allows us to access the definitions
from subsystems.targeting import Targeting
from commands2.button import CommandXboxController
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer, SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds
from subsystems.swerve_constants import DriveConstants as dc
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
class DriveByJoystickSubsystemTargeting(commands2.Command):
    def __init__(self, container, swerve: Swerve, targeting: Targeting, controller: CommandXboxController) -> None:
        super().__init__()
        self.setName('drive_by_joystick_subsystem_targeting')
        
        # -----------------------------------------------------------
        # 1. Subsystems & Dependencies
        # -----------------------------------------------------------
        self.container = container
        self.swerve = swerve
        self.targeting = targeting
        self.addRequirements(self.swerve)
        self.controller: CommandXboxController = controller

        # -----------------------------------------------------------
        # 2. Configuration & State
        # -----------------------------------------------------------
        self.field_oriented = True
        self.last_tracking_on = False

        # -----------------------------------------------------------
        # 3. Input Processing (Debouncers, Limiters)
        # -----------------------------------------------------------
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        
        # Use constants for slew rates to ensure tuning consistency
        self.drive_limiter = SlewRateLimiter(dc.kDriverSlewRate)
        self.strafe_limiter = SlewRateLimiter(dc.kDriverSlewRate)
        self.turbo_limiter = SlewRateLimiter(dc.kTurboSlewRate)
        
        # Rotation limiters
        self.manual_rot_limiter = SlewRateLimiter(dc.kDriverSlewRate)

        # -----------------------------------------------------------
        # 4. NetworkTables
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
        pass

    def execute(self) -> None:
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
        desired_fwd, desired_strafe, raw_vector = self._process_translation(inputs, slowmode_multiplier)

        # --- Rotation Processing ---
        raw_rot = -inputs['right_x']
        
        if inputs['tracking_on']:
            # Rising Edge: Reset targeting state
            if not self.last_tracking_on:
                self.targeting.reset_state()
            
            # Calculate commanded velocity for targeting
            # This is better than measured velocity because it has 0 lag and works in Sim
            if self.field_oriented:
                # Convert Field-Relative commands (desired_fwd/strafe) back to Robot-Relative ChassisSpeeds
                # We rotate by -RobotAngle to get Robot-Relative
                field_vel = Translation2d(desired_fwd, desired_strafe).rotateBy(-inputs['robot_pose'].rotation())
                vx = field_vel.X() * dc.kMaxSpeedMetersPerSecond
                vy = field_vel.Y() * dc.kMaxSpeedMetersPerSecond
            else:
                vx = desired_fwd * dc.kMaxSpeedMetersPerSecond
                vy = desired_strafe * dc.kMaxSpeedMetersPerSecond

            # Delegate to subsystem
            desired_rot = self.targeting.calculate_target_rotation(
                inputs['robot_pose'], 
                ChassisSpeeds(vx, vy, 0)
            )
            
        elif self.last_tracking_on: # Falling Edge
            # Reset manual limiter to the LAST TARGETING SPEED to ensure smooth handoff
            # This prevents the robot from jerking if stick is 0 but robot is spinning fast
            manual_rot = raw_rot * angular_slowmode_multiplier
            self.manual_rot_limiter.reset(self.targeting.last_rot_output)
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
            self.js_dv1_x_pub.set(math.fabs(raw_vector.X()))
            self.js_dv1_y_pub.set(math.fabs(raw_vector.Y()))
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

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)