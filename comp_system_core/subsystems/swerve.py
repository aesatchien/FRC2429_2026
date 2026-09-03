import math
import typing
import time

import ntcore
import wpilib
from commands2 import Subsystem

from wpilib import Alliance, DataLogManager, DriverStation, MatchState, RobotBase, SmartDashboard, Timer
from wpimath import SlewRateLimiter
from wpimath import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d
from wpimath import (ChassisVelocities, SwerveModuleVelocity, SwerveDrive4Kinematics)
from wpimath import SwerveDrive4PoseEstimator
from wpimath import PIDController

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, PathPlannerPath
from pathplannerlib.config import ModuleConfig, RobotConfig

import constants
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants as dc, AutoConstantsSwerve as ac, ModuleConstants as mc, RateLimiters as rl
from helpers.utilities import compare_motors
import helpers.apriltag_utils as atu
from subsystems.quest import Questnav

class Swerve (Subsystem):
    def __init__(self, questnav:Questnav) -> None:
        super().__init__()
        self.counter = constants.DrivetrainConstants.k_counter_offset
        self.questnav = questnav  #  pass in the questnav subsystem so we can query it in periodic

        # ----------  initialize swerve modules  ----------
        self.swerve_modules = []
        sd = dc.swerve_dict
        module_config = ([  # has to be LF, RF, LB, RB in that order
            [sd['LF']['driving_can'], sd['LF']['turning_can'], sd['LF']['port'], sd['LF']['turning_offset'], 'lf'],
            [sd['RF']['driving_can'], sd['RF']['turning_can'], sd['RF']['port'], sd['RF']['turning_offset'], 'rf'],
            [sd['LB']['driving_can'], sd['LB']['turning_can'], sd['LB']['port'], sd['LB']['turning_offset'], 'lb'],
            [sd['RB']['driving_can'], sd['RB']['turning_can'], sd['RB']['port'], sd['RB']['turning_offset'], 'rb'],
        ])
        for drive_id, turn_id, enc_port, offset, label in module_config:
            self.swerve_modules.append(SwerveModule(
                drivingCANId=drive_id, turningCANId=turn_id, encoder_analog_port=enc_port,
                turning_encoder_offset=offset, label=label))
        self.frontLeft, self.frontRight, self.rearLeft, self.rearRight = self.swerve_modules

        # let's make sure we're getting the right properties in the swerves.  describe() works
        # whichever vendor is underneath, so this still prints on a mixed Kraken/REV module.
        lf_drive, lf_turn = self.frontLeft.describe()
        compare_motors(lf_drive, lf_turn, name_a='LF DRIVE', name_b='LF TURN')

        # ---------- set up gyro   ----------
        # SystemCore has an IMU on the board, so there is no navX any more.  Two differences
        # that matter and are easy to get wrong:
        #   1. every OnboardIMU angle is in RADIANS.  navX was in degrees.  The wrappers
        #      below convert, so the rest of the code still speaks degrees.
        #   2. OnboardIMU is counter-clockwise-positive, the normal WPILib convention.  The
        #      navX was clockwise-positive, which is the only reason kGyroReversed existed.
        #      It is now False - see the note in swerve_constants.
        # There is no isCalibrating() to wait on, and no setAngleAdjustment(), so the
        # adjustment reset_gyro() used to hand the sensor is kept here in software instead.
        self.gyro = wpilib.OnboardIMU(dc.k_imu_mount_orientation)
        self.gyro.resetYaw()  # we boot up at zero degrees
        self.gyro_angle_adjustment = 0.0  # degrees, replaces navX setAngleAdjustment()
        self.gyro_calibrated = False

        # ---------- timer and variables for checking if we should be using pid on rotation ----------
        self.keep_angle = 0.0  # the heading we try to maintain when not rotating
        self.keep_angle_timer = Timer()
        self.keep_angle_timer.start()
        self.keep_angle_timer.reset()
        self.keep_angle_pid = PIDController(0.015, 0, 0)  # todo: put these in constants.  allow 1% stick per degree
        self.keep_angle_pid.enableContinuousInput(-180, 180)  # using the gyro's yaw is b/w -180 and 180
        self.last_rotation_time = 0
        self.time_since_rotation = 0
        self.last_drive_time = 0
        self.time_since_drive = 0

        # ---------- rate limiters  ----------  # TODO - centralize all tis
        self.fwd_magLimiter = SlewRateLimiter(rl.default_forward_slew_rate)
        self.strafe_magLimiter = SlewRateLimiter(rl.default_strafe_slew_rate)
        self.rotLimiter = SlewRateLimiter(rl.default_rotation_slew_rate)

        # ---------- brownout mode ----------
        self.brownout_mode = False  # toggled by driver button; reduces drive motor current limits

        # ---------- see if the asymmetry in the controllers is an issue for AJ  - 20250311 CJH ----------
        # update this in calibrate_joystick, and use in drive_by_joystick
        self.thrust_calibration_offset = 0
        self.strafe_calibration_offset = 0

        # ---------- pose estimator  ----------
        self.pose_estimator = SwerveDrive4PoseEstimator(dc.kDriveKinematics,
                                 Rotation2d.fromDegrees(self.get_gyro_angle()),                                                        self.get_module_positions(),
                                    initialPose=Pose2d(constants.k_start_x, constants.k_start_y,
                                    Rotation2d.fromDegrees(self.get_gyro_angle())))

        # ---------- Vision / NT  ----------
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.use_CJH_apriltags = constants.k_use_CJH_tags  # down below we decide which one to use in the periodic method
        
        self.camera_names = [config['topic_name'] for config in constants.CameraConstants.k_cameras.values() if config['type'] == 'tags']
        self.pose_subscribers = [self.inst.getDoubleArrayTopic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 7) for cam in self.camera_names]
        self.count_subscribers = [self.inst.getDoubleTopic(f"/Cameras/{cam}/tags/targets").subscribe(0) for cam in self.camera_names]

        # -------------  Pathplanner section --------------
        robot_config = RobotConfig.fromGUISettings()
        self._check_pathplanner_config(robot_config)

        AutoBuilder.configure(
                pose_supplier=self.get_pose,
                reset_pose=self.resetOdometry,
                robot_relative_speeds_supplier=self.get_relative_speeds,
                output=self.drive_robot_relative,
                controller=ac.k_pathplanner_holonomic_controller,
                robot_config=robot_config,
                should_flip_path=self.flip_path,
                drive_subsystem=self
        )

        self.automated_path = None

        # ------------- Advantagescope section -------------
        if constants.k_enable_logging:
            DataLogManager.start()  # start wpilib datalog for AdvantageScope
            DriverStation.startDataLog(DataLogManager.getLog())  # Record both DS control and joystick data
            # URCL is optional and imported lazily, so a missing package is a message rather
            # than an import error at module scope.  Toggle constants.k_enable_urcl.
            if constants.k_enable_urcl:
                try:
                    import urcl
                    urcl.URCL.start()  # unofficial REV logger for AdvantageScope
                    print('  started URCL (REV device logging)')
                except ImportError:
                    print('  *** k_enable_urcl is True but robotpy-urcl is not installed - skipping ***')
                    print('      (no 2027 build exists yet; set constants.k_enable_urcl = False to silence)')
            # URCL only sees REV devices.  If any Krakens are on the bus they log through
            # Phoenix's own SignalLogger, which writes .hoot files AdvantageScope opens separately.
            if dc.k_drive_vendor == 'ctre' or dc.k_turn_vendor == 'ctre':
                from phoenix6 import SignalLogger
                SignalLogger.start()
                print('  started Phoenix SignalLogger (.hoot) alongside URCL')

        # pre-allocate all the keys for speed
        self._init_networktables()

    def _check_pathplanner_config(self, robot_config) -> None:
        """Shout if deploy/pathplanner/settings.json disagrees with our own constants.

        settings.json is edited in the PathPlanner GUI and feeds AutoBuilder's feedforward and
        acceleration model.  Nothing links it to this file, so it silently drifts - it was
        describing a NEO on L3 with 0.048 m wheels while the code ran a Vortex on L2 with
        0.0508 m wheels, which had PathPlanner planning for a drivetrain 24% faster than ours.
        Switching drive vendors makes that worse, because driveMotorType has to change too.
        """
        module = robot_config.moduleConfig
        # DCMotor.freeSpeed is rad/s AFTER withReduction(gearing), i.e. at the wheel
        expected_free_rad_s = mc.kDrivingMotorFreeSpeedRps * math.tau / mc.kDrivingMotorReduction
        expected_motor = {'rev': 'a REV motor (vortex/NEO)', 'ctre': 'krakenX60 or krakenX60FOC'}[dc.k_drive_vendor]

        checks = [
            ('wheel radius (m)', module.wheelRadiusMeters, mc.kWheelDiameterMeters / 2, 0.001),
            ('drive current limit (A)', module.driveCurrentLimit, mc.kDrivingMotorCurrentLimit, 0.5),
            ('wheel free speed (rad/s)', module.driveMotor.freeSpeed, expected_free_rad_s, 0.5),
        ]
        bad = [(name, got, want) for name, got, want, tol in checks if abs(got - want) > tol]
        if bad:
            print('*' * 78)
            print(f'*** deploy/pathplanner/settings.json DISAGREES with swerve_constants '
                  f'(drive vendor = {dc.k_drive_vendor}) ***')
            for name, got, want in bad:
                print(f'***   {name:26s} settings.json says {got:8.4f}   code says {want:8.4f}')
            print(f'***   driveMotorType should be {expected_motor}')
            print('***   Fix it in the PathPlanner GUI - autos will follow paths with the wrong feedforward.')
            print('*' * 78)

    # ------------- NetworkTables  ------------
    def _init_networktables(self):
        swerve_prefix = constants.swerve_prefix
        status_prefix = constants.status_prefix

        # ------------- NetworkTables Publishers (Efficiency) -------------
        # Pre-allocate publishers to avoid hash lookups and string creation in periodic loops

        # let the coprocessors know if we have decided to do tag averaging
        self.allow_tag_averaging_pub = self.inst.getBooleanTopic(f"/Cameras/tag_averaging").publish()

        # Use StructPublisher for Pose2d - extremely efficient and works natively with AdvantageScope
        self.pose_pub = self.inst.getStructTopic(f"{swerve_prefix}/drive_pose", Pose2d).publish()
        # self.pose_pub = self.inst.getDoubleArrayTopic(f"{swerve_prefix}/drive_pose").publish()  # legacy GUI dashboard

        self.drive_x_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/drive_x").publish()
        self.drive_y_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/drive_y").publish()
        self.drive_theta_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/drive_theta").publish()

        self.navx_angle_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/_navx_angle").publish()
        self.navx_yaw_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/_navx_yaw").publish()
        self.navx_raw_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/_navx").publish()
        self.keep_angle_pub = self.inst.getDoubleTopic(f"{swerve_prefix}/keep_angle").publish()
        self.ypr_pub = self.inst.getDoubleArrayTopic(f"{swerve_prefix}/_navx_YPR").publish()

        # Debugging publishers - pre-allocate list to avoid f-string creation in loop
        module_names = ['LF', 'RF', 'LB', 'RB']  # TODO - just save this order somewhere and reuse it
        self.abs_enc_pubs = [self.inst.getDoubleTopic(f"{swerve_prefix}/absolute_{name}").publish() for name in module_names]
        self.angles_pub = self.inst.getDoubleArrayTopic(f"{swerve_prefix}/_angles").publish()

        self.brownout_mode_pub = self.inst.getBooleanTopic(f"{status_prefix}/brownout_mode").publish()
        self.brownout_mode_pub.set(self.brownout_mode)  # publish initial False



    # ----------  pose and odometry function definitions ----------
    def get_pose(self) -> Pose2d:
        # return the pose of the robot  TODO: update the dashboard here?
        return self.pose_estimator.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(), pose)

    def validate_odometry(self, pose: Pose2d) -> bool:
        # check if the pose is physically possible for the robot center to be at
        hw = constants.FieldConstants.k_robot_width / 2.0
        if not (hw <= pose.X() <= constants.FieldConstants.k_field_length - hw): return False
        if not (hw <= pose.Y() <= constants.FieldConstants.k_field_width - hw): return False
        return True

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, rate_limited: bool, keep_angle:bool=True) -> None:
        """Method to drive the robot using joystick info.
        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        # ORDER MATTERS.  keep_angle first, because it needs the RAW driver intent to decide
        # whether we are deliberately rotating or drifting; then rate limiting, on whatever
        # keep_angle decided to send.  It used to be the other way round, which meant the
        # rate-limited rotation was computed and then immediately overwritten by
        # perform_keep_angle() - so default_rotation_slew_rate had never once applied.
        if keep_angle:
            rot = self.perform_keep_angle(xSpeed, ySpeed, rot)  # the 1706 keep angle routine

        if rate_limited:
            xSpeedCommanded = self.fwd_magLimiter.calculate(xSpeed)
            ySpeedCommanded = self.strafe_magLimiter.calculate(ySpeed)
            rotation_commanded = self.rotLimiter.calculate(rot)
        else:
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            rotation_commanded = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * dc.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * dc.kMaxSpeedMetersPerSecond
        rotDelivered = rotation_commanded * dc.kMaxAngularSpeed

        # create the swerve state array depending on if we are field relative or not
        # 2027: ChassisSpeeds.fromFieldRelativeSpeeds() no longer exists.  Build the
        # velocities in the field frame, then rotate them into the robot frame.  Verified
        # numerically identical to the 2026 call.
        chassis_velocities = ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        if fieldRelative:
            chassis_velocities = chassis_velocities.toRobotRelative(Rotation2d.fromDegrees(self.get_angle()))
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleVelocities(chassis_velocities)

        # normalize wheel speeds so we do not exceed our speed limit
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelVelocities(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)


    # ----------  keepangle function definitions ----------
    def reset_keep_angle(self):
        self.last_rotation_time = self.keep_angle_timer.get()  # reset the rotation time
        self.last_drive_time = self.keep_angle_timer.get()  # reset the drive time

        new_angle = self.get_angle()
        print(f'  resetting keep angle from {self.keep_angle:.1f} to {new_angle:.1f}', flush=True)
        self.keep_angle = new_angle

    def perform_keep_angle(self, xSpeed, ySpeed, rot):  # update rotation if we are drifting when trying to drive straight
        output = rot  # by default we will return rot unless it needs to be changed
        if math.fabs(rot) > dc.k_inner_deadband:  # we are actually intending to rotate
            self.last_rotation_time = self.keep_angle_timer.get()
        if math.fabs(xSpeed) > dc.k_inner_deadband or math.fabs(ySpeed) > dc.k_inner_deadband:
            self.last_drive_time = self.keep_angle_timer.get()

        self.time_since_rotation = self.keep_angle_timer.get() - self.last_rotation_time
        self.time_since_drive = self.keep_angle_timer.get() - self.last_drive_time

        if self.time_since_rotation < 0.5:  # (update keep_angle until 0.5s after rotate command stops to allow rotate to finish)
            self.keep_angle = self.get_angle()
        elif math.fabs(rot) < dc.k_inner_deadband and self.time_since_drive < 0.25:  # stop keep_angle .25s after you stop driving
            output = self.keep_angle_pid.calculate(self.get_angle(), self.keep_angle)  # 2024 real, can we just use YAW always?
            output = output if math.fabs(output) < 0.2 else 0.2 * math.copysign(1, output)  # clamp at 0.2

        return output

    def get_brownout_mode(self) -> bool:
        return self.brownout_mode

    def set_brownout_mode(self, enabled: bool) -> None:
        """Toggle reduced drive motor current limits to protect a weak battery.
        Normal: mc.kDrivingMotorCurrentLimit (60A).  Brownout: mc.kDrivingMotorBrownoutCurrentLimit (40A).
        """
        self.brownout_mode = enabled
        self.brownout_mode_pub.set(self.brownout_mode)
        limit = mc.kDrivingMotorBrownoutCurrentLimit if enabled else mc.kDrivingMotorCurrentLimit
        print(f'Brownout mode {"ON" if enabled else "OFF"}: setting drive current limit to {limit}A')
        for module in self.swerve_modules:
            module.set_drive_current_limit(limit)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [45, -45, -45, 45]
        # print('Setting Swerve X')
        for angle, swerve_module in zip(angles, self.swerve_modules):
            # setDesiredState filters you out if your speed is less than a threshold, so gotta give it a small amount
            swerve_module.setDesiredState(SwerveModuleVelocity(0.005, Rotation2d.fromDegrees(angle)))

    def set_straight(self):
        """Sets the wheels straight so we can push the robot."""
        angles = [0, 0, 0, 0]
        for angle, swerve_module in zip(angles, self.swerve_modules):
            # setDesiredState filters you out if your speed is less than a threshold, so gotta give it a small amount
            swerve_module.setDesiredState(SwerveModuleVelocity(0.005, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: typing.Tuple[SwerveModuleVelocity]) -> None:
        desiredStates = SwerveDrive4Kinematics.desaturateWheelVelocities(desiredStates, dc.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_module_states(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getState() for m in self.swerve_modules]

    #  -------------  gyro functions  ----------

    def get_raw_angle(self):  # never reversed value for using PIDs on the heading
        # getAngleZ() is the accumulating angle, the counterpart of navX getAngle().
        # Adding the adjustment here matches navX, where setAngleAdjustment() moved
        # getAngle() but deliberately left getYaw() alone.
        return math.degrees(self.gyro.getAngleZ()) + self.gyro_angle_adjustment

    def get_gyro_angle(self):  # if necessary reverse the heading for swerve math
        # note this does add in the current offset
        return -self.get_raw_angle() if dc.kGyroReversed else self.get_raw_angle()

    def get_angle(self):  # if necessary reverse the heading for swerve math
        # used to be get_gyro_angle but LHACK changed it 12/24/24 so we don't have to manually reset gyro anymore
        return self.get_pose().rotation().degrees()

    def get_yaw(self):  # helpful for determining nearest heading parallel to the wall
        # but you should probably never use this - just use get_angle to be consistent
        yaw = math.degrees(self.gyro.getYaw())  # OnboardIMU is radians
        return -yaw if dc.kGyroReversed else yaw

    def get_pitch(self):
        # Which physical axis is pitch depends on how the SystemCore is mounted - this
        # assumes k_imu_mount_orientation.  Check it before trusting the number.
        pitch_offset = 0
        return math.degrees(self.gyro.getAngleY()) - pitch_offset

    def get_roll(self):
        roll_offset = 0
        return math.degrees(self.gyro.getAngleX()) - roll_offset

    def reset_gyro(self, adjustment=None):
        # OnboardIMU has no setAngleAdjustment(), so the offset lives in this class and
        # get_raw_angle() applies it.  Same behaviour, one layer higher up.
        self.gyro.resetYaw()
        self.gyro_angle_adjustment = adjustment if adjustment is not None else 0.0
        self.reset_keep_angle()

    #  -------------  simulation helpers  ----------
    def get_desired_swerve_module_states(self) -> list[SwerveModuleVelocity]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]


    #  -------------  METHODS PATHPLANNER NEEDS  ----------
    def get_relative_speeds(self):
        return dc.kDriveKinematics.toChassisVelocities(self.get_module_states())

    def drive_robot_relative(self, chassis_speeds: ChassisVelocities, feedforwards):
        # required for the pathplanner lib's pathfollowing based on chassis speeds
        # idk if we need the feedforwards
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleVelocities(chassis_speeds)
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelVelocities(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    def flip_path(self):  # pathplanner needs a function to see if it should mirror a path
        if MatchState.getAlliance() == Alliance.BLUE:
            return False
        else:
            return True
    # -------------- END PATHPLANNER STUFF  --------------


    # -------------- periodic and periodic helpers --------------
    def periodic(self) -> None:
        self.counter += 1
        ts = Timer.getTimestamp()
        current_pose = self.get_pose()  # Optimization: Cache pose to avoid recalculating it below

        self._update_vision_measurements(current_pose, ts)
        self._update_odometry(ts)
        
        if self.counter % 10 == 0:
            self._update_dashboard(current_pose, ts)

    def _update_vision_measurements(self, current_pose, ts):

        # QuestNav Logic - since swerve was instantiated with the questnav, it should use it just fine
        if self.questnav.use_quest and self.questnav.quest_has_synched and self.questnav.is_quest_connected() and self.counter % 4 == 0:
            quest_accepted = self.questnav.is_pose_accepted()
            quest_pose = self.questnav.quest_pose # Quest subsystem now exposes the robot-relative pose directly
            delta_pos = current_pose.translation().distance(quest_pose.translation())
            if delta_pos < 4 and quest_accepted:  # if the quest is way off, we don't want to update from it
                if self.validate_odometry(quest_pose):
                    # Calculate the packet latency in its native time domain
                    if self.questnav.mock_questnav:
                        latency_sec = (ntcore._now() / 1e6) - self.questnav.quest_pose_timestamp
                    else:
                        latency_sec = time.time() - self.questnav.quest_pose_timestamp
                    
                    # Apply that latency to the FPGA Match Time (protect against negative clock jitter)
                    quest_fpga_timestamp = ts - max(0.0, latency_sec)

                    self.pose_estimator.addVisionMeasurement(quest_pose, quest_fpga_timestamp, constants.DrivetrainConstants.k_pose_stdevs_large)
                elif self.counter % 100 == 0:
                    print(f"*** QuestNav update REJECTED: {quest_pose.X():.2f}, {quest_pose.Y():.2f} is outside field limits! ***")

        
        # AprilTag Logic
        if self.use_CJH_apriltags:
            for count_subscriber, pose_subscriber in zip(self.count_subscribers, self.pose_subscribers):
                if count_subscriber.get() > 0:  # use this camera's tag
                    atomic_data = pose_subscriber.getAtomic()
                    tag_data = atomic_data.value  # 7 items - id, tx, ty, tz, rx, ry, rz
                    timestamp_us = atomic_data.time
                    
                    # Check for stale tags (e.g. > 0.5s latency) using NT timestamp
                    # 500,000 microseconds = 0.5 seconds
                    latency_us = ntcore._now() - timestamp_us
                    if latency_us > 500000:
                        continue

                    tag_id = int(tag_data[0])
                    # make sure it's not a training tag not intended for odometry (returns None if not in layout)
                    if atu.layout.getTagPose(tag_id) is None and tag_data[0] != -1:
                        continue

                    tx, ty, tz = tag_data[1], tag_data[2], tag_data[3]
                    rx, ry, rz = tag_data[4], tag_data[5], tag_data[6]
                    tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()

                    use_tag = constants.k_use_CJH_tags  # can disable this in constants
                    # abs() because the gyro rate is SIGNED - without it we rejected motion-blurred
                    # tags when spinning one direction and accepted them at any rate spinning the other.
                    use_tag = False if abs(math.degrees(self.gyro.getGyroRateZ())) > 90 else use_tag  # no more than n deg/s while using a tag

                    if use_tag:
                        if self.validate_odometry(tag_pose):
                            tag_latency_sec = max(0.0, latency_us / 1_000_000.0)
                            tag_fpga_timestamp = ts - tag_latency_sec

                            # Standard deviations tell the pose estimator how much to "trust" this measurement.
                            # Smaller numbers = more trust. We trust vision more when disabled and stationary.
                            # Units are (x_meters, y_meters, rotation_radians).
                            sdevs = constants.DrivetrainConstants.k_pose_stdevs_large if wpilib.RobotState.isEnabled() else constants.DrivetrainConstants.k_pose_stdevs_disabled
                            
                            self.pose_estimator.addVisionMeasurement(tag_pose, tag_fpga_timestamp, sdevs)
                        elif self.counter % 100 == 0:
                            print(f"*** AprilTag {tag_id} update REJECTED: {tag_pose.X():.2f}, {tag_pose.Y():.2f} is outside field limits! ***")

    def _update_odometry(self, ts):
        if RobotBase.isReal():
            self.pose_estimator.updateWithTime(ts, Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(),)
            
        # Clamp the pose estimator to the physical field boundaries to prevent wheel-slip creep
        pose = self.get_pose()
        hw = constants.FieldConstants.k_robot_width / 2.0
        clamped_x = max(hw, min(constants.FieldConstants.k_field_length - hw, pose.X()))
        clamped_y = max(hw, min(constants.FieldConstants.k_field_width - hw, pose.Y()))
        
        if clamped_x != pose.X() or clamped_y != pose.Y():
            clamped_pose = Pose2d(clamped_x, clamped_y, pose.rotation())
            self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(), clamped_pose)
            if self.counter % 25 == 0:
                print(f"*** Odometry clamped to field bounds at {ts:.2f}s. Attempted pose: X={pose.X():.2f}, Y={pose.Y():.2f} ***")

    def _update_dashboard(self, pose, ts):

        # Send the struct (replaces the arrays). AdvantageScope detects this automatically.
        self.pose_pub.set(pose)
        # self.pose_pub.set([pose.X(), pose.Y(), pose.rotation().degrees()])  # legacy version

        # allow averaging to AprilTags on coprocessors when disabled OR when we are sitting still
        if constants.k_allow_tag_averaging and wpilib.RobotState.isDisabled():
            self.allow_tag_averaging_pub.set(True)
        else:
            self.allow_tag_averaging_pub.set(False)

        # Scalars (if you still need them for a specific dashboard layout)
        self.drive_x_pub.set(pose.X())
        self.drive_y_pub.set(pose.Y())
        self.drive_theta_pub.set(pose.rotation().degrees())

        self.navx_raw_pub.set(self.get_angle())
        self.navx_yaw_pub.set(self.get_yaw())
        self.navx_angle_pub.set(self.get_gyro_angle())
        self.keep_angle_pub.set(self.keep_angle)

        # post yaw, pitch, roll so we can see what is going on with the climb
        ypr = [self.get_yaw(), self.get_pitch(), self.get_roll(), self.gyro.getRotation2d().degrees()]
        self.ypr_pub.set(ypr)


        if constants.k_swerve_debugging_messages:
            angles = [m.get_turn_motor_position() for m in self.swerve_modules]
            absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
            
            for pub, val in zip(self.abs_enc_pubs, absolutes):
                pub.set(val)
            
            self.angles_pub.set(angles)
