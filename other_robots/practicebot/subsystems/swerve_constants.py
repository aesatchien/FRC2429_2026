"""
Swerve Drive Constants

This file contains all the physical, kinematic, and electrical constants for the Swerve Drive subsystem.
It also includes the configuration for the REV SparkMax/Flex motor controllers.

--- Control Loop Hierarchy and Tuning "Strengths" ---

1. Driver Input Layer (The "Feel") - Located in DriveByJoystickSwerveTargeting.py
   - Response Curve (sqrt): Makes robot less sensitive near center, ramps to full speed quickly.
   - Slow Mode Multiplier: 0.2 (Base) to 1.0 (Turbo). Caps speed at 20% unless trigger pulled.
   - Manual Slew Rate: 3.0 units/sec. Limits how fast rotation command changes manually.

2. Targeting Layer (The "Brain") - Located in DriveByJoystickSwerveTargeting.py & TargetingConstants
   - Lookahead Time (kTargetingLookaheadS): 0.9s. Aims at future target position to compensate for lag.
   - Rotation PID (kTeleopRotationPID.kP): 0.8. "Spring constant" pulling nose to target.
   - Physics Feedforward: 1.0. Calculates exact angular velocity needed for tangential speed.
   - Static Friction (kTeleopRotationkS): 0.05. Minimum output to break friction.
   - Tracking Slew Rate: Disabled/High. Allows auto-aim to react instantly.

3. Kinematics Layer (The "Limiter") - Located in DriveConstants
   - Max Speed: 4.75 m/s. Ceiling for translation.
   - Max Angular Speed: 0.75 * 2pi rad/s. Ceiling for rotation.
   - Acceleration Limit: 5.0 (100%/0.2s). Prevents tipping/brownouts.

4. Motor Control Layer (The "Muscle") - Located in ModuleConstants
   - Drive Feedforward: 1/FreeSpeed. Open loop control providing ~95% of power.
   - Drive PID: 0.0. Currently disabled.
   - Turning PID: 0.3. Stiffness of wheel angle servo.
"""

import math
from pathplannerlib.auto import PathConstraints
from pathplannerlib.controller import PPHolonomicDriveController
import wpilib
from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from rev import SparkMax, SparkFlex, SparkFlexConfig, SparkMaxConfig
from pathplannerlib.config import DCMotor, PIDConstants

import constants


class DriveConstants:
    """
    Global constants for the Swerve Drive subsystem.
    Includes robot dimensions, speed limits, and hardware configuration.
    """

    # ==========================================
    # Robot Identification & Controller Type
    # ==========================================
    # Configuration is handled via ACTIVE_CONFIG dictionary below.


    # ==========================================
    # Speed & Acceleration Limits
    # ==========================================
    # Note that these are not the maximum possible speeds, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.75  # Sanjith started at 3.7, 4.25 was Haochen competition, 4.8 is full out on NEOs
    kMaxAngularSpeed = 0.75 * math.tau # 0.5 * math.tau  # radians per second was 0.5 tau through AVR - too slow
    # TODO: actually figure out what the total max speed should be - vector sum?
    kMaxTotalSpeed = 1.1 * math.sqrt(2) * kMaxSpeedMetersPerSecond  # sum of angular and rotational, should probably do hypotenuse
    
    # set the acceleration limits used in driving using the SlewRateLimiter tool
    kMagnitudeSlewRate = 5  # hundred percent per second (1 = 100%)
    kRotationalSlewRate = 5  # hundred percent per second (1 = 100%)
    kDriverSlewRate = 3  # Slew rate for manual driver control (units/sec)
    kAutoSlewRate = 2    # Slew rate for autonomous PID correction (units/sec)
    kTurboSlewRate = 10  # Slew rate for turbo mode trigger (units/sec)
    
    # Input Deadbands
    k_inner_deadband = 0.10  # use deadbands for joystick transformations and keepangle calculations
    k_outer_deadband = 0.95  # above this you just set it to 1 - makes going diagonal easier

    # Reporting
    k_swerve_state_messages = True # these currently send the pose data to the sim - keep them on

    # ==========================================
    # Physical Dimensions & Kinematics
    # ==========================================
    robot_chassis = 27.0  # in
    mk4i_offset = 2.5  # in

    kTrackWidth = units.inchesToMeters(robot_chassis - 2 * mk4i_offset)  # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(robot_chassis - 2 * mk4i_offset)   # Distance between front and back wheels on robot

    # kinematics gets passed [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]
    # Front left is X+Y+, Front right is + -, Rear left is - +, Rear right is - - (otherwise odometery is wrong)
    # this should be left as the convention, so match the above.  Then take care of turning issues with the
    # INVERSION OF THE TURN OR DRIVE MOTORS, GYRO and ABSOLUTE ENCODERS
    swerve_orientation = [(1, 1), (1, -1), (-1, 1), (-1, -1)]  # MAKE SURE ANGLE ENCODERS ARE CCW +
    kModulePositions = [
        Translation2d(swerve_orientation[0][0]*kWheelBase / 2, swerve_orientation[0][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[1][0]*kWheelBase / 2, swerve_orientation[1][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[2][0]*kWheelBase / 2, swerve_orientation[2][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[3][0]*kWheelBase / 2, swerve_orientation[3][1]*kTrackWidth / 2),
    ]
    # set up kinematics object for swerve subsystem
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # ==========================================
    # Hardware Configuration (Inversions, Encoders)
    # ==========================================
    # which motors need to be inverted - depends on if mounted on top (True) or bottom (False)
    # THIS IS THE FIRST CHECK - DRIVE WITH DPAD (robot relative) AND MAKE SURE DIRECTION IS CORRECT
    k_drive_motors_inverted = False  # drive forward and reverse correct?  If not, invert this.
    # THIS IS THE SECOND CHECK - HOW DO YOU TEST IT?
    # k_turn_motors_inverted = True  # True for 2023 - motors below are true, above are false
    # incorrect gyro inversion will make the pose odometry have the wrong sign on rotation
    # IF DRIVE MOTORS ARE CORRECT AND TURN MOTORS ARE CORRECT, THEN CCW IS POSITIVE OR YOU REVERSE GYRO?
    kGyroReversed = True  # False for 2023 (was upside down), True for 2024/2025
    # used in the swerve modules themselves to reverse the direction of the analog encoder
    # note turn motors and analog encoders must agree - or you go haywire
    k_reverse_analog_encoders = False  # False for 2024 and probably always.

    # max absolute encoder value on each wheel they seem to stop at 0.99 for some reason - 20230322 CJH
    k_analog_encoder_abs_max = 1.0 # 0.990  # can we just leave it as 1?

    # we pass this next one to the analog potentiometer object to determine the full range
    # IN RADIANS to feed right to the AnalogPotentiometer on the module
    k_analog_encoder_scale_factor = math.tau * 1 / k_analog_encoder_abs_max  # 1.011 * 2pi  # so have to scale back up to be b/w 0 and 1
    sf = k_analog_encoder_scale_factor

    # ==========================================
    # CAN IDs and Offsets
    # ==========================================
    
    PRACTICE_CONFIG = {
        'robot_id': 'practice',
        'controller_cls': SparkMax,
        'config_cls': SparkMaxConfig,
        'free_speed_rpm': 5676,
        'modules': {
            'LF': {'driving_can': 21, 'turning_can': 20, 'port': 3, 'turning_offset': sf * 0.498},
            'LB': {'driving_can': 23, 'turning_can': 22, 'port': 1, 'turning_offset': sf * 0.113},
            'RF': {'driving_can': 25, 'turning_can': 24, 'port': 2, 'turning_offset': sf * 0.091},
            'RB': {'driving_can': 27, 'turning_can': 26, 'port': 0, 'turning_offset': sf * 0.466}
        },
        'inversions': {'drive_motors_inverted': False, 'turn_motors_inverted': True}
    }

    COMP_CONFIG = {
        'robot_id': 'comp',
        'controller_cls': SparkFlex,
        'config_cls': SparkFlexConfig,
        'free_speed_rpm': 6784,
        'modules': {
            'LF': {'driving_can': 21, 'turning_can': 20, 'port': 3, 'turning_offset': sf * 0.057},
            'LB': {'driving_can': 23, 'turning_can': 22, 'port': 1, 'turning_offset': sf * 0.432},
            'RF': {'driving_can': 25, 'turning_can': 24, 'port': 2, 'turning_offset': sf * 0.071},
            'RB': {'driving_can': 27, 'turning_can': 26, 'port': 0, 'turning_offset': sf * 0.035}
        },
        'inversions': {'drive_motors_inverted': False, 'turn_motors_inverted': True}
    }

    # Select the active configuration based on constants.py
    if constants.k_swerve_config == "practice":
        ACTIVE_CONFIG = PRACTICE_CONFIG
    elif constants.k_swerve_config == "comp":
        ACTIVE_CONFIG = COMP_CONFIG
    else:
        raise ValueError(f'k_swerve_config "{constants.k_swerve_config}" must be one of [comp, practice]')

    # Aliases for compatibility
    k_robot_id = ACTIVE_CONFIG['robot_id']
    k_drive_controller_type = ACTIVE_CONFIG['controller_cls']
    swerve_dict = ACTIVE_CONFIG['modules']
    swerve_motor_inversions = ACTIVE_CONFIG['inversions']

    # Encoder Alignment Test Mode
    analog_encoder_test_mode = False  #  set this to test the wheel alignment
    if analog_encoder_test_mode:
        print(f'YOU ARE IN ENCODER ALIGNMENT TEST MODE -- DO NOT DRIVE!!!')
        # read the raw numbers from the encoders so we can write them all down for a given robot
        k_analog_encoder_scale_factor = 1.0  # override so we get the raw reading between 0 and 1
        for key in ['LF', 'RF', 'LB', 'RB'] :
            swerve_dict[key]['turning_offset'] = 0
    else:
        pass


class NeoMotorConstants:
    kFreeSpeedRpm = DriveConstants.ACTIVE_CONFIG['free_speed_rpm']

class ModuleConstants:
    """
    Constants for individual Swerve Modules.
    Includes gearing, PID gains, and SparkMax/Flex configurations.
    """

    # ==========================================
    # Gearing & Conversions
    # ==========================================
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 4 * 0.0254  #  0.1016  =  four inches
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = 6.75 #8.14 #6.75  # From MK4i website, L2  #  From (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0  # meters per second

    k_turning_motor_gear_ratio = 150/7  #  not needed when we switch to absolute encoder of 150/7
    kTurningEncoderPositionFactor = math.tau / k_turning_motor_gear_ratio # radian
    kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0  # radians per second

    # ==========================================
    # PID & Feedforward
    # ==========================================
    kDrivingP = 0
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps  # CJH tested 3/19/2023, works ok  - 0.2235
    kDrivingMinOutput = -0.96 # why is this here?
    kDrivingMaxOutput = 0.96
    k_smartmotion_max_velocity = 3  # m/s
    k_smartmotion_max_accel = 2  # m/s/s

    kTurningP = 0.3 #  CJH tested this 3/19/2023  and 0.25 was good.  Used in the wpilib PID controller, not rev
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    # ==========================================
    # Electrical & Current Limits
    # ==========================================
    # 2024 0414 CJH - 80A allows the drive motors to pull WAY too much and we brown out (AVR)
    kDrivingMotorCurrentLimit = 60  # amp - set to 50 for worlds to make sure no brownouts - maybe 60 will still be safe
    kTurningMotorCurrentLimit = 40  # amp

    # ==========================================
    # SparkMax/Flex Configurations
    # ==========================================
    k_driving_config = DriveConstants.ACTIVE_CONFIG['config_cls']()
    k_driving_config.inverted(DriveConstants.swerve_motor_inversions['drive_motors_inverted'])
    k_driving_config.closedLoop.pidf(p=0, i=0, d=0, ff=1/kDriveWheelFreeSpeedRps)
    k_driving_config.closedLoop.minOutput(-0.96)
    k_driving_config.closedLoop.maxOutput(0.96)
    k_driving_config.closedLoop.IZone(0.001)
    k_driving_config.closedLoop.maxMotion.maxVelocity(3)
    k_driving_config.closedLoop.maxMotion.maxAcceleration(2)
    k_driving_config.setIdleMode(idleMode=SparkFlexConfig.IdleMode.kBrake)
    k_driving_config.smartCurrentLimit(stallLimit=kDrivingMotorCurrentLimit, freeLimit=kDrivingMotorCurrentLimit, limitRpm=5700)
    k_driving_config.voltageCompensation(12)
    k_driving_config.encoder.positionConversionFactor((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) # meters
    k_driving_config.encoder.velocityConversionFactor((kWheelDiameterMeters * math.pi) / ( kDrivingMotorReduction * 60)) # meters per second
    # k_driving_config.closedLoop.pidf(0, 0, 0, 0.01)

    # note: we don't use any spark pid or ff for turning
    k_turning_config = DriveConstants.ACTIVE_CONFIG['config_cls']()
    k_turning_config.inverted(DriveConstants.swerve_motor_inversions['turn_motors_inverted'])
    k_turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_turning_config.smartCurrentLimit(stallLimit=kTurningMotorCurrentLimit, freeLimit=kTurningMotorCurrentLimit, limitRpm=5700)
    k_turning_config.voltageCompensation(12)

    # nor do we use this encoder-- we configure it "just to watch it if we need to for velocities, etc."
    k_turning_config.encoder.positionConversionFactor(math.tau/k_turning_motor_gear_ratio) # radian
    k_turning_config.encoder.velocityConversionFactor(math.tau/(k_turning_motor_gear_ratio * 60)) # radians per second


class AutoConstants:
    """
    Constants for Autonomous operation and PathPlanner.
    """

    k_pathplanner_translation_pid_constants = PIDConstants(kP=6, kI=0, kD=0)
    k_pathplanner_rotation_pid_constants = PIDConstants(kP=4, kI=0, kD=0)  # no longer negative when swerve correct

    k_pathplanner_holonomic_controller = PPHolonomicDriveController(
            translation_constants=k_pathplanner_translation_pid_constants,
            rotation_constants=k_pathplanner_rotation_pid_constants,
    )

    k_pathfinding_constraints = PathConstraints(
            maxVelocityMps=3,
            maxAccelerationMpsSq=6,  # this was at 6 for all comps - CJH lowered it to 4 for old batteries 20251006
            maxAngularVelocityRps=2*math.pi,  # radians per second
            maxAngularAccelerationRpsSq=4*math.pi,  # radians per second squared
            nominalVoltage=12,
    )

    # used as end conditions in auto to pose / pid to point
    k_rotation_tolerance = Rotation2d(math.radians(2))
    k_translation_tolerance_meters = 2 / 100


class TargetingConstants:
    """
    Constants for AutoToPose and Joystick Targeting.
    Centralizes PID gains and tolerances for targeting logic.
    """
    # AutoToPoseClean uses 0.7, Joystick uses 0.8.
    kAutoRotationPID = PIDConstants(0.7, 0.0, 0.0)
    kTeleopRotationPID = PIDConstants(0.8, 0.0, 0.0)
    
    # Translation PIDs for AutoToPose
    kAutoTranslationPID = PIDConstants(1.2, 0.1, 0.0)
    
    # Tolerances (mirrored from AutoConstants for now, but can be tuned separately)
    k_rotation_tolerance = AutoConstants.k_rotation_tolerance
    k_translation_tolerance_meters = AutoConstants.k_translation_tolerance_meters
    k_teleop_rotation_kS = 0.05 # Minimum output to overcome friction (static friction feedforward)
    k_targeting_lookahead_s = 0.9 # Lookahead time in seconds to anticipate robot motion (lag compensation)
    k_teleop_rotation_kf = 1.0 # Physics feedforward gain. 1.0 is exact, >1.0 overdrives for lag.
