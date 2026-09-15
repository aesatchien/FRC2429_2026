"""
The "ghost robot": draws the goal pose and shot line that the auto-to-pose commands publish
under constants.auto_prefix onto the field, and clears them a couple of seconds after the
command ends.  Pure display - it reads NT topics the commands already write and touches
nothing in the robot.  Pumped from MyRobot.simulation_periodic().
"""

import ntcore
import wpilib
from wpimath import Pose2d

import constants
from helpers import dashboard


class GhostRobot:
    k_linger_seconds = 2.0

    def __init__(self):
        self.inst = ntcore.NetworkTableInstance.get_default()
        auto_prefix = constants.auto_prefix
        self.auto_active_sub = self.inst.get_boolean_topic(f"{auto_prefix}/robot_in_auto").subscribe(False)
        self.goal_pose_sub = self.inst.get_struct_topic(f"{auto_prefix}/goal_pose", Pose2d).subscribe(Pose2d())
        self.shot_line_sub = self.inst.get_struct_array_topic(f"{auto_prefix}/shot_line", Pose2d).subscribe([])

        self.target_object = dashboard.field_object("Target")
        self.shotline_object = dashboard.field_object("ShotLine")
        self.last_update_time = 0.0

    def update(self) -> None:
        now = wpilib.Timer.get_timestamp()
        if self.auto_active_sub.get():
            self.target_object.set_pose(self.goal_pose_sub.get())
            self.shotline_object.set_poses(self.shot_line_sub.get())
            self.last_update_time = now
        elif now - self.last_update_time > self.k_linger_seconds:
            self.target_object.set_poses([])
            self.shotline_object.set_poses([])
