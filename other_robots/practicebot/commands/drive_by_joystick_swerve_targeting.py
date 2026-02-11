
import math
import commands2
import wpilib
import ntcore

import constants
from subsystems.swerve import Swerve  # allows us to access the definitions
from commands2.button import CommandXboxController
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer, SlewRateLimiter
from subsystems.swerve_constants import DriveConstants as dc
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

        # -----------------------------------------------------------
        # 4. Targeting Setup
        # -----------------------------------------------------------
        self.bHubLocation = Translation2d(4.74, 4.05)
        self.rHubLocation = Translation2d(12.1, 4.05)

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
        pass

    def execute(self) -> None:
        # -----------------------------------------------------------
        # 1. READ INPUTS
        # -----------------------------------------------------------
        # Robot Pose for targeting
        robot_pose = self.swerve.get_pose()
        
        # Joystick Inputs (using getHID to avoid overruns)
        hid = self.controller.getHID()
        right_trigger_value = hid.getRightTriggerAxis()
        robot_oriented_value = hid.getLeftBumperButton()
        left_y = hid.getLeftY()
        left_x = hid.getLeftX()
        right_x = hid.getRightX()
        
        # Alliance Color
        alliance = wpilib.DriverStation.getAlliance()

        # -----------------------------------------------------------
        # 2. CALCULATE
        # -----------------------------------------------------------
        
        # --- 2a. Targeting Calculations ---
        robot_location = robot_pose.translation()
        if (abs(self.rHubLocation - robot_location) < abs(self.bHubLocation - robot_location)):
            hub_vector = self.rHubLocation - robot_location
        else:
            hub_vector = self.bHubLocation - robot_location
        robot_to_hub_angle = hub_vector.angle()
        # TODO - implement angle usage

        # --- 2b. Drive Mode Calculations ---
        # Turbo / Slow Mode
        turbo = self.turbo_limiter.calculate(right_trigger_value**2)
        slowmode_multiplier = 0.2 + 0.8 * turbo
        angular_slowmode_multiplier = 0.5 + 0.5 * turbo

        # Field Oriented vs Robot Oriented
        if self.robot_oriented_debouncer.calculate(robot_oriented_value):
            self.field_oriented = False
        else:
            self.field_oriented = True

        # --- 2c. Joystick Pre-processing ---
        # Apply calibration offsets and negation
        joystick_fwd = -(left_y - self.swerve.thrust_calibration_offset)
        joystick_strafe = -(left_x - self.swerve.strafe_calibration_offset)
        joystick_rot = -right_x

        # Create raw vector for reporting and processing
        raw_vector = Translation2d(joystick_fwd, joystick_strafe)

        # --- 2d. Deadbanding & Shaping ---
        # Rotational Deadband
        if abs(joystick_rot) < dc.k_inner_deadband:
            joystick_rot = 0
            
        # Translational Deadband & Clipping
        processing_vector = raw_vector
        if processing_vector.norm() > dc.k_outer_deadband:
            processing_vector *= 1 / processing_vector.norm()
        elif processing_vector.norm() < dc.k_inner_deadband:
            processing_vector = Translation2d(0, 0)
            
        # Response Curve (Sqrt for sensitivity)
        # CJH added a 1.5 instead of 2.0 - may help with stuttering at low end 20250311
        processing_vector *= math.sqrt(processing_vector.norm())

        # --- 2e. Scaling ---
        processing_vector *= slowmode_multiplier
        desired_rot = joystick_rot * angular_slowmode_multiplier

        # --- 2f. Rate Limiting ---
        desired_fwd = self.drive_limiter.calculate(processing_vector.X())
        desired_strafe = self.strafe_limiter.calculate(processing_vector.Y())

        # --- 2g. Alliance Adjustment ---
        if alliance == wpilib.DriverStation.Alliance.kRed and self.field_oriented:
            desired_fwd *= -1
            desired_strafe *= -1

        # -----------------------------------------------------------
        # 3. ACT
        # -----------------------------------------------------------
        self.swerve.drive(
            xSpeed=desired_fwd,
            ySpeed=desired_strafe,
            rot=desired_rot,
            fieldRelative=self.field_oriented,
            rate_limited=self.rate_limited,
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


    def end(self, interrupted: bool) -> None:
        # probably should leave the wheels where they are?
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)
