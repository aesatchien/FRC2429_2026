# constants for the 2024 robot
from itertools import count
import math
import wpilib
import rev
from rev import ClosedLoopSlot, SparkClosedLoopController, SparkFlexConfig, SparkMax, SparkMaxConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from wpimath.units import inchesToMeters, lbsToKilograms
from typing import Union, List
from helpers.utilities import set_config_defaults

k_swerve_config = "comp"

# Generator for unique counter offsets
_counter = count(1)

# TODO - organize this better
k_enable_logging = True  # allow logging from Advantagescope (in swerve.py), but really we may as well start it here

# starting position for odometry (real and in sim)
k_start_x, k_start_y  = 2, 2

# ------------  joysticks and other input ------------
k_driver_controller_port = 0
k_co_driver_controller_port = 1

# should be fine to burn on every reboot, but we can turn this off
k_burn_flash = True

#  ----------  network tables organization - one source for truth in publishing
# systems outside the robot
camera_prefix = r'/Cameras'  # from the pis
quest_prefix = r'/QuestNav'  # putting this on par with the cameras as an external system

# systems inside/from the robot
status_prefix = r'/SmartDashboard/RobotStatus'  # the default for any status message
vision_prefix = r'/SmartDashboard/Vision'  # from the robot
swerve_prefix = r'/SmartDashboard/Swerve'  # from the robot
sim_prefix = r'/SmartDashboard/Sim'  # from the sim (still from the robot)
auto_prefix = r'/SmartDashboard/Auto'  # one place for all of our auto goals and temp variables
command_prefix = r'Command'  # SPECIAL CASE: the SmartDashboard.putData auto prepends /SmartDashboard to the key\
intake_prefix = r'/SmartDashboard/Intake'  # intake subsystem


k_swerve_debugging_messages = True
# multiple attempts at tags this year - TODO - use l/r or up/down tilted cameras again, gives better data
k_use_quest_odometry = True
k_use_photontags = False  # take tags from photonvision camera
k_use_CJH_tags = True  # take tags from the pis
k_allow_tag_averaging = True

k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True  # is there any reason for this at all?


class CameraConstants:
    #  ----------  camera configuration (may need its own class eventually)  ----------
    # Dictionary mapping Logical Name -> NetworkTables Camera Name in /Cameras
    # Each camera has a purpose, which can be 'tags' (apriltags) or 'orange' (hsv-filtered objects)
    # If one physical camera does both, we treat it as two cameras but with the same topic
    # ordering is nice to align with the IP and order they are on the pis, but not required
    # rotation angle is CCW positive from the front of the robot
    fov = 45  # sim testing fov, no effect on real robot yet

    k_practicebot_cameras = {
        'logi_front': {'topic_name': 'LogitechFront', 'type': 'tags', 'rotation': 0, 'fov': fov},
        'logi_front_hsv': {'topic_name': 'LogitechFront', 'type': 'hsv', 'label': 'yellow', 'rotation': 0, 'fov': fov},
        'logi_left': {'topic_name': 'LogitechLeft', 'type': 'tags', 'rotation': 90, 'fov': fov},
        'logi_left_hsv': {'topic_name': 'LogitechLeft', 'type': 'hsv', 'label': 'yellow', 'rotation': 90, 'fov': fov},
    }

    k_comp_cameras = {
        'arducam_right': {'topic_name': 'ArducamRight', 'type': 'tags', 'rotation': 0, 'fov': fov},
        'arducam_left': {'topic_name': 'ArducamLeft', 'type': 'tags', 'rotation': 0, 'fov': fov},
    }

    # for testing hsv pickup
    k_sim_cameras = {
        'logi_front_hsv': {'topic_name': 'LogitechFront', 'type': 'hsv', 'label': 'yellow', 'rotation': 0, 'fov': fov},
        'genius_low': {'topic_name': 'GeniusLow', 'type': 'tags', 'rotation': -90, 'fov': fov},
        'logi_left': {'topic_name': 'LogitechLeft', 'type': 'tags', 'rotation': 90, 'fov': fov},
        'logi_left_hsv': {'topic_name': 'LogitechLeft', 'type': 'hsv', 'label': 'yellow', 'rotation': 90, 'fov': fov},
    }

    k_cameras = k_sim_cameras

    # add local_tester.py's sim camera if in sim - allows for testing without pis
    if wpilib.RobotBase.isSimulation():
        k_cameras.update({'front_sim': {'topic_name': 'LocalTest', 'type': 'tags', 'rotation': 0, 'fov': fov},})


class SimConstants:
    k_counter_offset = next(_counter)
    k_cam_distance_limit = 4  # sim testing how far targets can be - usually 3 to 3.5m on the real cameras
    k_tag_visibility_angle = 60  # degrees, the angle from normal that the tag can be seen (90 means +/- 90 deg)

    k_print_config = True  # use for debugging the camera config

    k_disable_vision_sim = False  # Hard disable.  Set to stop all vision simulation (e.g. ONLY using real coprocessors)
    k_draw_camera_fovs = True  # Set to draw camera FOV triangles - should always want this
    k_use_external_cameras = False  # override the vision sim to only take targets from real cams - squashes blink_test
    k_do_blink_test = False  # Set to test dashboard connection handling (e.g. dropping camera connections)
    k_use_live_tags_in_sim = True  # Set to True to snap the robot's swerve sim to live AprilTag data


class VisionConstants:

    k_counter_offset = next(_counter)
    k_nt_debugging = False  # print extra values to NT for debugging
    k_pi_names = ["top_pi"]

    k_valid_tags = list(range(1, 23))

    k_print_config = True  # use for debugging the camera config


class QuestConstants:
    k_counter_offset = next(_counter)
    quest_to_robot = Transform2d(inchesToMeters(0), inchesToMeters(9.5), Rotation2d().fromDegrees(0))


class LedConstants:

    k_counter_offset = next(_counter)
    k_nt_debugging = False  # print extra values to NT for debugging
    k_led_count = 40  # correct as of 2025 0305
    k_led_count_ignore = 4  # flat ones not for the height indicator
    k_led_pwm_port = 0  # correct as of 2025 0305

class RobotStateConstants:

    k_counter_offset = next(_counter)
    k_nt_debugging = False  # print extra values to NT for debugging

class DrivetrainConstants:

    k_counter_offset = next(_counter)
    k_nt_debugging = False  # print extra values to NT for debugging
    # these are for the apriltags.  For the most part, you want to trust the gyro, not the tags for angle
    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
    # HIGH numbers = LOW trust  (~ big stdev = we don't trust it much) 2m is high, 0.1m is small
    k_pose_stdevs_large = (2, 2, 10)  # use when you don't trust the april tags - stdev x, stdev y, stdev theta
    k_pose_stdevs_disabled = (1, 1, 2)  # use when we are disabled to quickly get updates
    k_pose_stdevs_small = (0.1, 0.1, 10)  # use when you do trust the tags

class IntakeConstants:
    k_counter_offset = next(_counter)
    k_CANID_intake = 9  # IDK

    k_intake_config = SparkMaxConfig()
    k_intake_configs = [k_intake_config]
    k_test_rpm = 20  # pi * diameter roller / 60  to get inches per second
    k_fastest_rpm = 60

    k_intake_config.inverted(False)

    set_config_defaults(k_intake_configs)

class ShooterConstants:

    k_counter_offset = next(_counter)
    k_CANID_indexer = 5
    k_CANID_flywheel_left_leader, k_CANID_flywheel_right_follower = 7, 8  # left flywheel and follower
    k_CANID_turret = 9

    # FLYWHEEL
    k_flywheel_left_leader_config, k_flywheel_right_follower_config = SparkMaxConfig(), SparkMaxConfig()
    k_flywheel_configs = [k_flywheel_left_leader_config, k_flywheel_right_follower_config]
    k_test_speed = 4000
    k_fastest_speed = 6500
    k_test_rpm = 20
    k_fastest_rpm = 60

    k_flywheel_left_leader_config.inverted(False)  # have to check which way it spins for positive RPM
    # k_flywheel_right_follower.inverted(False)  # this is not necessary - it will get ignored

    # set up the followers
    k_flywheel_right_follower_config.follow(k_CANID_flywheel_left_leader, invert=False)  # depends on motor placement

    #setting brake, voltage compensation, and current limit for the flywheel motors
    set_config_defaults(k_flywheel_configs)

class ClimberConstants:
    k_counter_offset = next(_counter)