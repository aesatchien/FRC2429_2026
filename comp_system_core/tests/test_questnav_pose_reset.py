"""
Pin that QuestNav.set_pose() really delivers the pose reset to the headset.

WHY THIS EXISTS
---------------
set_pose() builds a protobuf command inside a broad try/except that only PRINTS on failure.
On a7 `quat.W()` (the 2026 spelling) became the property `quat.w`; the migration's rename
pattern skips capitalised accessors, so the call raised AttributeError, the except swallowed
it, and every pose reset silently never left the robot.  The driver station showed

    QuestNav error sending pose reset: 'Quaternion' object has no attribute 'W'

while quest.py carried on printing "Reset questnav" and marking the Quest as synced - so the
robot believed the headset was aligned to the field when it had never been told anything.

Because the failure is swallowed, "it did not raise" proves nothing.  This reads the command
back off NetworkTables and checks the quaternion that arrived.
"""

import math
import time

import ntcore
from wpimath import Pose3d, Rotation3d, Translation3d

import helpers.questnav.protos.generated.commands_pb2 as commands_pb2
from helpers.questnav.questnav import QuestNav

# must match the type string QuestNav publishes /QuestNav/request with, or NT delivers nothing
k_command_type = "proto:questnav.protos.commands.ProtobufQuestNavCommand"


def test_set_pose_publishes_a_pose_reset_with_the_right_quaternion():
    ntcore.NetworkTableInstance.get_default()     # local pub/sub is enough, no server needed
    quest = QuestNav()
    reader = quest.command_topic.subscribe(k_command_type, b"")

    yaw = 1.0                                     # not 0: w must differ from 1.0 to prove it was read
    quest.set_pose(Pose3d(Translation3d(1.5, 2.5, 0.1), Rotation3d(0.0, 0.0, yaw)))

    raw = b""
    for _ in range(50):                           # up to ~0.5 s for the local publish to land
        raw = reader.get()
        if raw:
            break
        time.sleep(0.01)
    assert raw, ("set_pose() published nothing - it swallowed an exception.  Run it by hand "
                 "and read the 'QuestNav error sending pose reset' line.")

    command = commands_pb2.ProtobufQuestNavCommand()
    command.ParseFromString(raw)
    assert command.type == commands_pb2.POSE_RESET

    target = command.pose_reset_payload.target_pose
    assert math.isclose(target.translation.x, 1.5)
    assert math.isclose(target.translation.y, 2.5)
    assert math.isclose(target.translation.z, 0.1)

    q = target.rotation.q
    assert math.isclose(q.w, math.cos(yaw / 2), abs_tol=1e-9)    # the field the bug dropped
    assert math.isclose(q.z, math.sin(yaw / 2), abs_tol=1e-9)
    assert math.isclose(q.x, 0.0, abs_tol=1e-9)
    assert math.isclose(q.y, 0.0, abs_tol=1e-9)
