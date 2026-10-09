"""
Pin that the Quest subsystem only calls itself synced once the headset has SHOWN the new pose.

WHY THIS EXISTS
---------------
quest_sync_odometry() used to set quest_has_synched = True the moment it sent the reset.  When the
reset never reached the headset (the `quat.W()` bug) the robot believed it was aligned to the
field and fused a pose that was off by metres.  It also accepted the first frames after a reset,
which can still carry the pre-reset pose, and cached is_tracking/is_connected before reading the
frames that update them.

These tests drive Questnav.periodic() with a fake headset client so each case is deterministic.
"""

import pytest
from wpimath import Pose2d, Pose3d, Rotation2d

from helpers import dashboard
from helpers.questnav.questnav import PoseFrame
from subsystems.quest import Questnav

dashboard.install()    # Questnav publishes its commands through the dashboard shim

TARGET = Pose2d(5.0, 4.0, Rotation2d.from_degrees(30))   # where the robot "is" for a sync
STALE = Pose2d(1.0, 1.0, Rotation2d.from_degrees(0))     # what the headset says before the reset lands


class FakeHeadset:
    """Just enough of helpers.questnav.QuestNav for Questnav.periodic()."""

    def __init__(self):
        self.frames = []
        self.read_yet = False          # tracking only becomes known once the frames are read
        self.connected = True
        self.send_ok = True            # False: set_pose() reports nothing was sent
        self.result = None             # what get_command_result() answers
        self.sent = []

    def command_periodic(self): pass
    def get_all_unread_pose_frames(self):
        self.read_yet = True
        frames, self.frames = self.frames, []
        return frames
    def is_connected(self): return self.connected and self.read_yet
    def is_tracking(self): return self.read_yet
    def set_pose(self, pose):
        self.sent.append(pose)
        return 7 if self.send_ok else None
    def get_command_result(self, command_id): return self.result
    def get_battery_percent(self): return 80
    def get_latency(self): return 5.0
    def get_tracking_lost_counter(self): return 0
    def get_frame_count(self): return 1


class FakeSub:
    def __init__(self, pose): self.pose = pose
    def get(self): return self.pose


@pytest.fixture
def quest():
    q = Questnav()
    q.mock_questnav = False                     # the real-headset code path, whatever constants say
    q.questnav = FakeHeadset()
    q.drive_pose_sub = FakeSub(TARGET)
    return q


def feed(q, robot_pose):
    """Queue one frame showing the headset at the Quest pose that corresponds to robot_pose."""
    q.questnav.frames.append(PoseFrame(
        quest_pose_3d=Pose3d(robot_pose.transform_by(q.quest_to_robot.inverse())),
        data_timestamp=0.0, app_timestamp=0.0, frame_count=1))


def test_sync_is_not_claimed_until_the_headset_shows_the_new_pose(quest):
    feed(quest, STALE)
    quest.periodic()
    quest.quest_sync_odometry()
    assert not quest.quest_has_synched, "sending the command must not mean synced"

    feed(quest, STALE)                          # headset still on the old pose
    quest.periodic()
    assert not quest.quest_has_synched
    assert not quest.is_pose_accepted(), "a pose that may predate the reset must not be fused"

    feed(quest, TARGET)                         # headset applied it
    quest.periodic()
    assert quest.quest_has_synched
    assert quest.pending_reset_id is None


def test_sync_that_never_lands_gives_up_and_stays_unsynced(quest):
    quest.k_reset_timeout = -1.0                # already expired
    feed(quest, STALE)
    quest.periodic()
    quest.quest_sync_odometry()
    feed(quest, STALE)
    quest.periodic()
    assert not quest.quest_has_synched
    assert quest.pending_reset_id is None       # free to try again


def test_headset_reporting_failure_fails_the_sync(quest):
    feed(quest, STALE)
    quest.periodic()
    quest.quest_sync_odometry()
    quest.questnav.result = False
    feed(quest, STALE)
    quest.periodic()
    assert not quest.quest_has_synched
    assert quest.pending_reset_id is None


def test_a_command_that_could_not_be_sent_is_not_a_sync(quest):
    feed(quest, STALE)
    quest.periodic()
    quest.questnav.send_ok = False
    quest.quest_sync_odometry()
    assert not quest.quest_has_synched
    assert quest.pending_reset_id is None


def test_second_sync_while_one_is_in_flight_is_ignored(quest):
    feed(quest, STALE)
    quest.periodic()                            # lets the fake headset report connected
    quest.quest_sync_odometry()
    quest.quest_sync_odometry()
    assert len(quest.questnav.sent) == 1


def test_frames_are_read_before_the_tracking_flag_is_checked(quest):
    # The fake only knows it is tracking once the frames have been read.  With the flags cached
    # first, this frame would be dropped as "not tracking" and the pose would never update.
    feed(quest, TARGET)
    quest.periodic()
    assert quest.quest_pose.translation().distance(TARGET.translation()) < 1e-6
    assert quest.was_tracking
