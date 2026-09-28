"""
Hardware-in-the-loop: teleport the simulated robot to wherever REAL hardware says it is.

Two sources, both optional:
  * a real camera on the network reporting a real AprilTag (constants.SimConstants.k_use_live_tags_in_sim)
  * a real Quest headset connected to the sim (constants.SimConstants.k_mock_questnav = False)

This does not fit the per-subsystem simulation model - it is not a mechanism, it is a way of
using bench hardware to drive the sim - so it lives here and is pumped from
MyRobot.simulation_periodic().  It moves only GROUND TRUTH (Swerve.sim_set_ground_truth).  It
never touches the pose estimator: the estimator learns where the robot is the same way it
does on the real robot, from the vision measurements Swerve.periodic() already consumes.
Under the old physics.py the estimator was force-reset to ground truth every loop, so a snap
here moved the estimate too; that coupling is gone on purpose.
"""

import ntcore
from wpimath import Pose3d, Rotation3d, Translation3d

import constants
import helpers.apriltag_utils


class HardwareInTheLoop:
    k_max_tag_age_us = 100_000   # 0.1 s - fresh enough to snap to, stale enough to ignore our own sim tags

    def __init__(self, container):
        self.container = container
        self.swerve = container.swerve
        self.questnav = container.questnav
        self.inst = ntcore.NetworkTableInstance.get_default()

        # Live tag subscribers.  Same topics Swerve subscribes to; separate subscriptions so
        # the two consumers cannot interfere with each other's get_atomic() bookkeeping.
        self.camera_names = [config['topic_name'] for config in constants.CameraConstants.k_cameras.values()
                             if config['type'] == 'tags']
        self.pose_subscribers = [self.inst.get_double_array_topic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 7)
                                 for cam in self.camera_names]
        self.count_subscribers = [self.inst.get_double_topic(f"/Cameras/{cam}/tags/targets").subscribe(0)
                                  for cam in self.camera_names]

    def update(self) -> None:
        if constants.SimConstants.k_use_live_tags_in_sim:
            self._snap_to_live_tags()

        q = self.questnav
        if (not constants.SimConstants.k_mock_questnav and q.use_quest and q.quest_has_synched
                and q.is_pose_accepted()):
            self._snap_to_quest()

    def _snap_to_live_tags(self) -> None:
        """If a live camera sees a tag, move the simulated robot to that location.
        Lets vision-based localisation be tested in the simulator with real hardware."""
        for count_sub, pose_sub in zip(self.count_subscribers, self.pose_subscribers):
            if count_sub.get() <= 0:
                continue
            atomic_data = pose_sub.get_atomic()
            tag_data = atomic_data.value
            timestamp_us = atomic_data.time

            # Freshness.  This is ALSO what keeps us from snapping to our own simulated tags:
            # vision_sim.py deliberately never writes the 'poses' topic, so a simulated camera
            # leaves it stale and it is ignored here.
            if ntcore._now() - timestamp_us >= self.k_max_tag_age_us:
                continue

            # a training tag that is not in the layout is not for odometry (get_tag_pose -> None)
            if helpers.apriltag_utils.layout.get_tag_pose(int(tag_data[0])) is None and int(tag_data[0]) != -1:
                return

            tx, ty, tz = tag_data[1], tag_data[2], tag_data[3]
            rx, ry, rz = tag_data[4], tag_data[5], tag_data[6]
            vision_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).to_pose2d()
            self.swerve.sim_set_ground_truth(vision_pose)
            return   # first valid tag wins, so two cameras cannot fight

    def _snap_to_quest(self) -> None:
        """If the physical Quest headset provides a tracking update, teleport the sim robot to match it."""
        self.swerve.sim_set_ground_truth(self.questnav.quest_pose)
