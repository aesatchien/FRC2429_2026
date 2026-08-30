"""
Swerve Drive Constants

This file contains all the physical, kinematic, and electrical constants for the Swerve Drive subsystem.
It also includes the motor controller configuration for both vendors - REV SparkMax/Flex
and CTRE Kraken X60/TalonFX.  Which one a given position uses is set per-config below and
resolved in subsystems/motors.py.

--- Control Loop Hierarchy: WHAT each layer is for ---

This map deliberately does NOT repeat any numbers.  It used to, and by mid-season four of
them were wrong (it claimed slow mode was 0.2, teleop rotation kP was 0.8, and max angular
speed was 0.75*tau, none of which had been true for months).  A comment that restates a
constant is a second copy that nothing checks.  Read the value off the constant itself;
read the *purpose* here.

1. Driver Input Layer (The "Feel") - commands/drive_by_joystick_subsystem_targeting.py
   - Response curve: raises stick magnitude to a power > 1, so small stick motions near
     center are gentle and the top of the throw still reaches full speed.
   - Slow-mode floor + turbo trigger: the floor caps speed for normal driving; squeezing
     the trigger interpolates from the floor up to 1.0.
   - Translation slew: bounds how fast the driver can change a translation command.

2. Targeting Layer (The "Brain") - subsystems/targeting.py & TargetingConstants
   - Time-of-flight lookahead: aims where the robot WILL be when the ball lands, not where
     it is now.  Iterated twice because ToF depends on the distance it is trying to find.
   - Rotation PID: the spring pulling the nose onto the hub.
   - Physics feedforward: the angular velocity needed to keep the nose on a target while
     translating past it.  Pure geometry - omega = (v x r) / |r|^2 - so 1.0 is "exact".
   - Static friction term: minimum output that actually breaks the drivetrain loose.

3. Kinematics Layer (The "Limiter") - DriveConstants
   - Max translation / angular speed: the ceilings we choose to allow, NOT what the
     hardware can do.  See kMaxSpeedMetersPerSecond for the measured hardware number.
   - Slew rates (RateLimiters below): bound acceleration to prevent tipping and brownouts.

4. Motor Control Layer (The "Muscle") - ModuleConstants
   - Drive feedforward 1/FreeSpeed: open-loop term that provides most of the power; the
     drive PID only trims the remainder.
   - Turning PID: stiffness of the wheel-angle servo.  Runs on the roboRIO against the
     analog absolute encoder, not on the Spark.
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
    # These are the speeds we ALLOW, not the speeds the hardware can reach.
    # Physical ceiling = free_rpm/60 * wheel_circumference / reduction, at 6.75:1 on 4in wheels:
    #     Vortex 6784 rpm -> 5.35 m/s      Kraken X60 6000 rpm -> 4.73 m/s
    # !! The comp bot is on Krakens now, so 4.75 is ABOVE the 4.73 the drivetrain can deliver.
    #    Nothing breaks - the closed loop just saturates - but the top of the stick does nothing
    #    and there is zero headroom. Lower it (4.2-4.5) or accept a permanently saturated full
    #    stick. Left alone for now because changing it changes driving feel. -- see kMaxTotalSpeed
    # History: Sanjith started at 3.7, 4.25 was the Haochen competition setting, 4.8 was full out
    # on NEOs.
    kMaxSpeedMetersPerSecond = 4.75
    kMaxAngularSpeed = 7 # 0.5 * math.tau  # radians per second was 0.5 tau through AVR - too slow
    kSlowModeCap = 0.35                   # translation slow-mode floor for targeting command
    kAngularSlowFloor = 0.5               # angular slow-mode floor for both joystick commands
    kJoystickLegacyTranslationFloor = 0.2 # translation slow-mode floor for drive_by_joystick_swerve only
    # our hardware can do 11.11 hertz =
    # TODO: actually figure out what the total max speed should be - vector sum?
    # Ceiling handed to SwerveDrive4Kinematics.desaturateWheelSpeeds().  NOTE this is 7.39, which
    # is far ABOVE what a module can physically do (4.73 m/s on Krakens, 5.35 on Vortexes), so
    # desaturation effectively never fires: full-forward + full-rotation asks two modules for
    # ~7.0 m/s, they clip individually instead of the set scaling down together, and the chassis
    # stops moving in the commanded direction.  The Kraken swap makes this worse, not better.
    # Left as-is because changing it changes driving feel - but it should become the measured
    # module top speed, which is the whole point of the function.
    kMaxTotalSpeed = 1.1 * math.sqrt(2) * kMaxSpeedMetersPerSecond
    
    # Acceleration limits: see RateLimiters class below for all SlewRateLimiter rates.
    
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
    
    # Each config names its motors per-position.  'drive_vendor'/'turn_vendor' are 'rev' or 'ctre'
    # and are read by subsystems/motors.py; the *_cls / *_free_speed_rpm entries only matter for
    # whichever positions are 'rev' (CTRE gearing and free speed live in ModuleConstants below).
    PRACTICE_CONFIG = {
        'robot_id': 'practice',
        'drive_vendor': 'rev',      'turn_vendor': 'rev',
        'drive_controller_cls': SparkMax,  'turn_controller_cls': SparkMax,
        'config_cls': SparkMaxConfig,
        'drive_free_speed_rpm': 5676,      # NEO
        'modules': {
            'LF': {'driving_can': 21, 'turning_can': 20, 'port': 3, 'turning_offset': sf * 0.498},
            'LB': {'driving_can': 23, 'turning_can': 22, 'port': 1, 'turning_offset': sf * 0.113},
            'RF': {'driving_can': 25, 'turning_can': 24, 'port': 2, 'turning_offset': sf * 0.091},
            'RB': {'driving_can': 27, 'turning_can': 26, 'port': 0, 'turning_offset': sf * 0.466}
        },
        'inversions': {'drive_motors_inverted': False, 'turn_motors_inverted': True}
    }

    # THE COMP BOT: Kraken X60 on the drive, SparkFlex on the turn.
    # CAN ids are unchanged from the Vortex era - CTRE and REV use different device-type fields
    # in the CAN arbitration id, so a TalonFX at 21 and a SparkFlex at 20 coexist on the roboRIO
    # bus exactly as they did before.
    COMP_CONFIG = {
        'robot_id': 'comp',
        'drive_vendor': 'ctre',     'turn_vendor': 'rev',
        'drive_controller_cls': None,       'turn_controller_cls': SparkFlex,
        'config_cls': SparkFlexConfig,      # still used to build the REV turn config
        'drive_free_speed_rpm': 6000,       # Kraken X60, trapezoidal.  5800 if you license FOC.
        'modules': {
            'LF': {'driving_can': 21, 'turning_can': 20, 'port': 3, 'turning_offset': sf * 0.672},  # .475 worked then got off then changed to .511
            'LB': {'driving_can': 23, 'turning_can': 22, 'port': 1, 'turning_offset': sf * 0.435},
            'RF': {'driving_can': 25, 'turning_can': 24, 'port': 2, 'turning_offset': sf * 0.081},
            'RB': {'driving_can': 27, 'turning_can': 26, 'port': 0, 'turning_offset': sf * 0.0335}
        },
        'inversions': {'drive_motors_inverted': False, 'turn_motors_inverted': True}
    }

    # ROLLBACK: the same comp bot with Vortexes back on the drive.  If a Kraken dies at an event
    # and you bolt a Vortex in, this is a one-word change in constants.k_swerve_config instead of
    # a code edit under pressure.  It shares COMP_CONFIG's modules and inversions, so the two
    # cannot drift apart - only the drive motor differs.  Remember settings.json's driveMotorType
    # has to go back to "vortex" too; the boot check in Swerve will shout at you if you forget.
    COMP_VORTEX_CONFIG = {
        'robot_id': 'comp_vortex',
        'drive_vendor': 'rev',      'turn_vendor': 'rev',
        'drive_controller_cls': SparkFlex,  'turn_controller_cls': SparkFlex,
        'config_cls': SparkFlexConfig,
        'drive_free_speed_rpm': 6784,       # NEO Vortex
        'modules': COMP_CONFIG['modules'],
        'inversions': COMP_CONFIG['inversions'],
    }

    # Select the active configuration based on constants.py
    k_all_configs = {'practice': PRACTICE_CONFIG, 'comp': COMP_CONFIG, 'comp_vortex': COMP_VORTEX_CONFIG}
    if constants.k_swerve_config not in k_all_configs:
        raise ValueError(f'k_swerve_config "{constants.k_swerve_config}" must be one of {sorted(k_all_configs)}')
    ACTIVE_CONFIG = k_all_configs[constants.k_swerve_config]

    # Aliases for compatibility
    k_robot_id = ACTIVE_CONFIG['robot_id']
    k_drive_vendor = ACTIVE_CONFIG['drive_vendor']
    k_turn_vendor = ACTIVE_CONFIG['turn_vendor']
    swerve_dict = ACTIVE_CONFIG['modules']
    swerve_motor_inversions = ACTIVE_CONFIG['inversions']

    # Encoder Alignment Test Mode
    analog_encoder_test_mode = False  #  set this to test the wheel alignment
    if analog_encoder_test_mode:
        print(f'YOU ARE IN ENCODER ALIGNMENT TEST MODE -- DO NOT DRIVE!!!')
        # read the raw numbers from the encoders so we can write them all down for a given robot
        k_analog_encoder_scale_factor = 1.0  # override so we get the raw reading between 0 and 1
        for key in ['LF', 'RF', 'LB', 'RB']:
            swerve_dict[key]['turning_offset'] = 0
    else:
        pass


class RateLimiters:
    """
    Centralized SlewRateLimiter rates for all swerve drive control modes.
    Units: fraction-of-max-speed per second  (1.0 = 100% of max speed per second).

    Three sections map to the three places where limiting is applied:
      1. Joystick commands — do their own limiting, then call swerve.drive(rate_limited=False)
      2. swerve.drive() fallback — active when rate_limited=True; used by commands that
         do not do their own limiting
      3. Autonomous pose commands — do their own (stricter) limiting, rate_limited=False

    Import alias convention:  from subsystems.swerve_constants import RateLimiters as rl
    """

    # ═══════════════════════════════════════════════════════════════════════════
    # Section 1 — Joystick Operations
    # ═══════════════════════════════════════════════════════════════════════════
    # Used in: commands/drive_by_joystick_subsystem_targeting.py  (primary teleop)
    #          commands/drive_by_joystick_swerve.py               (fallback / practice)
    # These commands call swerve.drive(rate_limited=False), so Section 2 does NOT apply.

    # Translation (fwd / strafe) input slew — limits how fast the driver can command acceleration
    driver_translation_slew_rate = 3.0    # units/sec

    # Manual rotation slew — limiter is created in targeting command but currently not applied
    # (see manual_rot_limiter in DriveByJoystickSubsystemTargeting)
    driver_rotation_slew_rate = 3.0       # units/sec

    # Turbo trigger input smoothing — prevents a lurch when the trigger is squeezed quickly
    turbo_input_slew_rate = 10.0          # units/sec

    # Afterburner button input smoothing
    afterburner_input_slew_rate = 20.0    # units/sec

    # ═══════════════════════════════════════════════════════════════════════════
    # Section 2 — Default Rate Limiting (swerve.py fallback)
    # ═══════════════════════════════════════════════════════════════════════════
    # Applied inside swerve.drive() only when rate_limited=True.
    # Callers on this path:
    #   commands/drive_by_velocity_swerve.py     — always rate_limited=True
    #   commands/auto_track_vision_target.py     — always rate_limited=True
    #   commands/drive_by_joystick_swerve.py     — when rate_limited=True; Section 1 still
    #                                              applies first, and dominates (rate 3 < rate 5)
    #   end() calls in all drive commands        — sending zeros; limiting is irrelevant

    default_forward_slew_rate = 4.5       # slightly softer than strafe (historically 0.9 × 5.0)
    default_strafe_slew_rate = 5.0
    default_rotation_slew_rate = 5.0

    # ═══════════════════════════════════════════════════════════════════════════
    # Section 3 — Autonomous Driving
    # ═══════════════════════════════════════════════════════════════════════════
    # Used in: commands/auto_to_pose_clean.py            (x, y, and rot limiters)
    #          commands/drive_to_pose_custom_control.py  (x, y, and rot limiters)
    # These commands call swerve.drive(rate_limited=False), so Section 2 does NOT apply.
    # Stricter than teleop (2.0 vs 3.0): PID outputs can be large step inputs that
    # would cause brownouts if not limited before reaching the motors.

    auto_translation_slew_rate = 2.0      # units/sec — applied to x and y PID outputs
    auto_rotation_slew_rate = 2.0         # units/sec — applied to rotation PID output


class NeoMotorConstants:
    # Free speed of whatever motor is on the DRIVE - Vortex, NEO or Kraken.  Kept under the old
    # name so nothing outside this file has to change; the turn motor's free speed is not used
    # anywhere (the RIO closes that loop) so there is only ever one number to track here.
    kFreeSpeedRpm = DriveConstants.ACTIVE_CONFIG['drive_free_speed_rpm']

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
    kDrivingMotorCurrentLimit = 50         # amp - normal operation
    kDrivingMotorBrownoutCurrentLimit = 40 # amp - reduced limit when battery can't deliver full current
    kTurningMotorCurrentLimit = 40         # amp

    # ==========================================
    # Controller classes for whichever positions are 'rev'
    # ==========================================
    k_drive_controller_cls = DriveConstants.ACTIVE_CONFIG['drive_controller_cls']
    k_turn_controller_cls = DriveConstants.ACTIVE_CONFIG['turn_controller_cls']

    # ==========================================
    # SparkMax/Flex Configurations
    # ==========================================
    k_driving_config = DriveConstants.ACTIVE_CONFIG['config_cls']()
    k_driving_config.inverted(DriveConstants.swerve_motor_inversions['drive_motors_inverted'])
    k_driving_config.closedLoop.pidf(p=0.02, i=0, d=0, ff=1/kDriveWheelFreeSpeedRps)
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

    # ==========================================
    # Kraken X60 / TalonFX Configuration  (drive only, for k_swerve_config = "comp_kraken")
    # ==========================================
    # Built unconditionally so the numbers are readable and diffable even on an all-REV robot;
    # nothing constructs a TalonFX unless a config actually says drive_vendor='ctre'.
    #
    # Where the gains came from.  REV closed-loop output is DUTY CYCLE and our conversion factors
    # put the setpoint in m/s, so REV `ff` has units of duty per m/s.  Phoenix `k_v` is VOLTS per
    # ROTATION-PER-SECOND.  Converting needs both a voltage scale and a length scale:
    #
    #     k_v = ff_rev * 12 V * circumference          and equivalently   12 V / wheel_free_rps
    #     k_p = kP_rev * 12 V * circumference
    #
    # Both routes agree, which is the check that the units are right.  These are DERIVED starting
    # points, not measured ones - run SysId before you trust them, and see k_kraken_ks below.
    k_kraken_free_speed_rpm = 6000           # X60 trapezoidal.  FOC is 5800 and a different curve.
    k_kraken_wheel_free_speed_rps = k_kraken_free_speed_rpm / 60 / kDrivingMotorReduction  # wheel rot/s
    k_kraken_kv = 12.0 / k_kraken_wheel_free_speed_rps                # ~0.810 V per wheel-rps
    k_kraken_kp = 0.02 * 12.0 * kWheelCircumferenceMeters             # ~0.0766, from REV kP=0.02
    k_kraken_ki = 0.0
    k_kraken_kd = 0.0
    # No REV equivalent exists - our Spark config has no static term at all, which is why
    # setDesiredState has to deadband below 0.002 m/s.  Measure this with SysId and the
    # deadband hack may become unnecessary.
    k_kraken_ks = 0.0

    # FOC needs a per-device Phoenix Pro licence.  Every Phoenix control request defaults to
    # enable_foc=True, so we have to say False explicitly until the licences are bought.
    # When you do buy them: set this True AND change k_kraken_free_speed_rpm to 5800.
    k_kraken_enable_foc = False
    k_kraken_canbus = 'rio'                  # no CANivore yet

    # REV gives one current knob; Phoenix gives two, and they are different quantities.
    #   supply  - protects the breaker and the battery.  This is what brownout mode moves.
    #   stator  - limits torque, and therefore wheel slip.  This is the traction knob.
    # Starting conservative: supply matches today's 50 A exactly, stator is only slightly above
    # so the swap does not hand the drivers a large torque change on day one.  A Kraken will
    # happily take 80-120 A stator; raise it deliberately after testing traction and brownouts.
    k_kraken_supply_current_limit = kDrivingMotorCurrentLimit   # 50 A, same as the Vortex today
    k_kraken_stator_current_limit = 60                          # amps

    # Guarded so a laptop without phoenix6 can still run the all-REV robot and the sim.
    # If a config asks for drive_vendor='ctre' without the package, motors.py raises a clear
    # error naming the pip command - we do not want an ImportError at module scope taking
    # down the whole robot program the way the apriltag layout path used to.
    try:
        from phoenix6.configs import TalonFXConfiguration as _TalonFXConfiguration
        from phoenix6.signals import InvertedValue as _InvertedValue, NeutralModeValue as _NeutralModeValue
        k_kraken_configs_built = True
    except ImportError:
        k_kraken_configs_built = False
        k_kraken_driving_config = None
        k_kraken_turning_config = None

    if k_kraken_configs_built:

        k_kraken_driving_config = _TalonFXConfiguration()

        # inverted(bool) -> an enum.  Same physical meaning, driven off the same dict.
        k_kraken_driving_config.motor_output.inverted = (
            _InvertedValue.CLOCKWISE_POSITIVE if DriveConstants.swerve_motor_inversions['drive_motors_inverted']
            else _InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        k_kraken_driving_config.motor_output.neutral_mode = _NeutralModeValue.BRAKE   # was setIdleMode(kBrake)
        k_kraken_driving_config.motor_output.duty_cycle_neutral_deadband = 0.0        # REV has none; match it

        # sensor_to_mechanism_ratio makes the device report WHEEL rotations.  The metres
        # conversion happens in motors.TalonDriveMotor, per CTRE's own recommendation.
        k_kraken_driving_config.feedback.sensor_to_mechanism_ratio = kDrivingMotorReduction

        k_kraken_driving_config.slot0.k_p = k_kraken_kp
        k_kraken_driving_config.slot0.k_i = k_kraken_ki
        k_kraken_driving_config.slot0.k_d = k_kraken_kd
        k_kraken_driving_config.slot0.k_v = k_kraken_kv
        k_kraken_driving_config.slot0.k_s = k_kraken_ks

        # was closedLoop.minOutput/maxOutput(+/-0.96) - i.e. +/- 0.96 * 12 V
        k_kraken_driving_config.voltage.peak_forward_voltage = 0.96 * 12
        k_kraken_driving_config.voltage.peak_reverse_voltage = -0.96 * 12

        k_kraken_driving_config.current_limits.supply_current_limit = k_kraken_supply_current_limit
        k_kraken_driving_config.current_limits.supply_current_limit_enable = True
        k_kraken_driving_config.current_limits.stator_current_limit = k_kraken_stator_current_limit
        k_kraken_driving_config.current_limits.stator_current_limit_enable = True

        # Deliberately NOT ported, and why:
        #   voltageCompensation(12)      - VelocityVoltage control is inherently compensated
        #   closedLoop.IZone(0.001)      - Phoenix slot configs have no IZone, and our kI is 0
        #   maxMotion.maxVelocity(3)     - dead on the REV side too; we command kVelocity, not MAXMotion
        #   maxMotion.maxAcceleration(2) - same
        #   PersistMode / k_burn_flash   - Phoenix configs persist in the device by default

        # Turn config, for the day a module goes all-CTRE.  Nothing builds this today.
        k_kraken_turning_config = _TalonFXConfiguration()
        k_kraken_turning_config.motor_output.inverted = (
            _InvertedValue.CLOCKWISE_POSITIVE if DriveConstants.swerve_motor_inversions['turn_motors_inverted']
            else _InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        k_kraken_turning_config.motor_output.neutral_mode = _NeutralModeValue.BRAKE
        k_kraken_turning_config.feedback.sensor_to_mechanism_ratio = k_turning_motor_gear_ratio
        k_kraken_turning_config.current_limits.supply_current_limit = kTurningMotorCurrentLimit
        k_kraken_turning_config.current_limits.supply_current_limit_enable = True


class AutoConstantsSwerve:
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
    #  ROTATION PIDs - one set per consumer.  (Do not write the values in this comment;
    #  the previous version said 0.7 / 0.8 and had been wrong for most of the season.)
    kAutoRotationPID = PIDConstants(0.7, 0.0, 0.0)  # auto_to_pose.py
    kTeleopRotationPID = PIDConstants(0.5, 0.0, 0.06) # targeting.py
    
    # TRANSLATION PIDs for AutoToPose
    kAutoTranslationPID = PIDConstants(0.8, 0.1, 0.0)
    
    # Tolerances (mirrored from AutoConstants for now, but can be tuned separately)
    k_rotation_tolerance = AutoConstantsSwerve.k_rotation_tolerance
    k_translation_tolerance_meters = AutoConstantsSwerve.k_translation_tolerance_meters
    k_teleop_rotation_kS = 0.05 # Minimum output to overcome friction (static friction feedforward)
    k_teleop_rotation_kf = 1.0 # Physics feedforward gain. 1.0 is exact, >1.0 overdrives for lag.
    kShotAccuracyToleranceMeters = 0.5 # Shot must land within this distance of the target center to be "OK"
    kTargetingVelocityDeadband = 0.1 # m/s - ignore velocity below this for prediction to prevent jitter
    kMinTargetDistance = 0.25 # meters - avoid division by zero in feedforward calculation
