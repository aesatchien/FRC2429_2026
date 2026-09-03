import math
from pathlib import Path

import robotpy_apriltag
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import constants
from constants import FieldConstants as fc, AutoConstants as cac

# This data is initialized once when the module is first imported.
# TODO -  robotpy_apriltag has no k2026RebuiltWelded field yet, so we ship the layout ourselves - 20260118 CJH
#layout = robotpy_apriltag.AprilTagFieldLayout.loadField(robotpy_apriltag.AprilTagField.k2026RebuiltWelded)
# Resolve relative to this file, not the working directory.  The old version hardcoded
# '2026-rebuilt-welded_json' (sim, CWD-relative) and '/home/lvuser/py/...' (robot); either one
# going stale is an ImportError at module scope, which takes down the whole robot program.
k_tag_layout_file = Path(__file__).resolve().parent.parent / '2026-rebuilt-welded_json'
layout = robotpy_apriltag.AprilTagFieldLayout(str(k_tag_layout_file))


# Pre-calculate tag positions for plotting or other uses
tag_positions = {tag_id: layout.getTagPose(tag_id).translation().toTranslation2d()
                 for tag_id in range(17, 23) if layout.getTagPose(tag_id) is not None}

def get_tag_distance(tag_id, current_pose):
    """ Return the distance from the current pose to the given tag ID """
    tag_pose = layout.getTagPose(tag_id).toPose2d()
    distance = current_pose.translation().distance(tag_pose.translation())
    return distance

def auto_reflect_pose(robot_pose:Pose2d, goal_pose:Pose2d, alliance, is_shooting=False):
    print(f"alliance: {alliance}, robot_pose: {robot_pose}, goal_pose: {goal_pose}")
    if alliance == wpilib.DriverStation.Alliance.kRed:
        # x and theta for lower half red
        theta = math.pi - goal_pose.rotation().radians()
        x = fc.k_field_length - goal_pose.X()
    else:
        # x and theta for lower half blue
        theta = goal_pose.rotation().radians()
        x = goal_pose.X()

    # reflect y about the center if we're on the top half of the field
    y = fc.k_field_width - goal_pose.Y() if robot_pose.Y() > fc.k_field_width / 2 else goal_pose.Y()

    # for a shooting pose, flip theta if we're on the top half of the field, so we face the hub
    if is_shooting and robot_pose.Y() > fc.k_field_width / 2:
        theta = -theta

    # print(f"pose.Y =={pose.Y():.1f}")

    return Pose2d(x, y, Rotation2d(theta))

def get_nearest_tag(current_pose: Pose2d, tags: list[int]) -> int:
    """ Return the ID of the nearest tag in `tags` to a given pose.

    Callers pass the tag list explicitly.  The 2025 version took a `destination` string
    and looked the list up internally, which meant every new destination needed an edit here.
    """
    poses = {tag: layout.getTagPose(tag) for tag in tags}
    valid = {tag: pose.toPose2d() for tag, pose in poses.items() if pose is not None}
    if not valid:
        raise ValueError(f'none of the tags {tags} exist in the {k_tag_layout_file.name} layout')

    return min(valid, key=lambda tag: current_pose.translation().distance(valid[tag].translation()))


def mirror_for_alliance(pose: Pose2d, alliance=None) -> Pose2d:
    """ Rotate a blue-origin pose 180 degrees about field center when we are on red.

    This is THE alliance mirror for the whole codebase.  It used to be copy-pasted into
    three helpers here plus AutoToPoseClean._determine_target_pose; if you find a fourth
    copy, delete it and call this instead.

    Note this is a rotation about center (the 2025+ "rotationally symmetric field" rule),
    NOT a reflection.  auto_reflect_pose() above is a different operation - it mirrors a
    play about the field's horizontal centerline so the same auto works top or bottom.
    """
    if alliance is None:
        alliance = wpilib.DriverStation.getAlliance()
    if alliance != wpilib.DriverStation.Alliance.kRed:
        return pose

    field_center = Translation2d(fc.k_field_length / 2, fc.k_field_width / 2)
    return pose.rotateAround(point=field_center, rot=Rotation2d(math.pi))
