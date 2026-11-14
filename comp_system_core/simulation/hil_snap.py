"""
Hardware-in-the-loop: teleport the simulated robot to wherever REAL hardware says it is.

Two sources, both optional, both switched by constants.SimConstants.k_snap_to_real_sensors:
  * a real camera on the network reporting a real AprilTag
  * a real Quest headset connected to the sim (SimConstants.k_quest_source = 'real')
Neither can fire on simulated data, so it is safe to leave on: the mock Quest is never
snapped to, and with SimConstants.k_camera_source = 'sim' only the cameras marked 'external'
in CameraConstants are listened to - the rest are the camera sim's own, and it publishes tag
poses made from ground truth.

This does not fit the per-subsystem simulation model - it is not a mechanism, it is a way of
using bench hardware to drive the sim - so it lives here and is pumped from
MyRobot.simulation_periodic().  It moves only GROUND TRUTH (Swerve.sim_set_ground_truth).  It
never touches the pose estimator: the estimator learns where the robot is the same way it
does on the real robot, from the vision measurements Swerve.periodic() already consumes.
Under the old physics.py the estimator was force-reset to ground truth every loop, so a snap
here moved the estimate too; that coupling is gone on purpose.
"""

from helpers.nt_time import nt_age_seconds
import ntcore
from wpimath import Pose3d, Rotation3d, Translation3d

import constants
import helpers.apriltag_utils


class HardwareInTheLoop:
    k_max_tag_age_s = 0.1   # fresh enough to snap to, stale enough to ignore our own sim tags

    def __init__(self, container):
        self.container = container
        self.swerve = container.swerve
        self.questnav = container.questnav
        self.inst = ntcore.NetworkTableInstance.get_default()

        # Live tag subscribers.  Same topics Swerve subscribes to; separate subscriptions so
        # the two consumers cannot interfere with each other's get_atomic() bookkeeping.
        sim_owns_cameras = constants.SimConstants.k_camera_source == 'sim'
        self.camera_names = [config['topic_name'] for config in constants.CameraConstants.k_cameras.values()
                             if config['type'] == 'tags' and (config.get('external') or not sim_owns_cameras)]
        self.pose_subscribers = [self.inst.get_double_array_topic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 7)
                                 for cam in self.camera_names]
        self.count_subscribers = [self.inst.get_double_topic(f"/Cameras/{cam}/tags/targets").subscribe(0)
                                  for cam in self.camera_names]

    def update(self) -> None:
        if not constants.SimConstants.k_snap_to_real_sensors:
            return

        self._snap_to_live_tags()

        q = self.questnav  # last, so a real Quest wins over a real tag
        if not q.mock_questnav and q.use_quest and q.quest_has_synched and q.is_pose_accepted():
            self._snap_to_quest()

    def _snap_to_live_tags(self) -> None:
        """If a live camera sees a tag, move the simulated robot to that location.
        Lets vision-based localisation be tested in the simulator with real hardware."""
        for count_sub, pose_sub in zip(self.count_subscribers, self.pose_subscribers):
            if count_sub.get() <= 0:
                continue
            atomic_data = pose_sub.get_atomic()
            tag_data = atomic_data.value

            # Freshness - a camera that has stopped reporting must not hold the robot in place.
            # seconds via helpers.nt_time - a7 NT timestamps are nanoseconds
            if nt_age_seconds(atomic_data.time) >= self.k_max_tag_age_s:
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
