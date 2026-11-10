"""
A delivered pose reset is cleared off /QuestNav/request - but not before it is delivered.

WHY THIS EXISTS
---------------
NT keeps the last value of /QuestNav/request.  When the QuestNav app restarts or resumes (a
double-tap into passthrough, the GUI's adb restart) it reads that value again and re-applies
the old POSE_RESET, teleporting the headset back to the last sync point.  QuestNav now
overwrites the request with a no-op once the reset has been delivered.

The first try at this (feat/quest_clear_cmd, Apr 2026) cleared on the first pose frame after
set_pose, which can be 20 ms later.  NT only sends the newest value each transmit cycle, so
that could replace the reset before the headset ever saw it.  These tests pin both halves:
the reset survives until the headset answers it (or k_clear_after_s), then it is gone.
"""

import time

import ntcore
from wpimath import Pose3d, Rotation3d, Translation3d

import helpers.questnav.protos.generated.commands_pb2 as commands_pb2
from helpers.questnav.questnav import QuestNav

k_command_type = "proto:questnav.protos.commands.ProtobufQuestNavCommand"


def _request(reader) -> commands_pb2.ProtobufQuestNavCommand:
    raw = b""
    for _ in range(50):                  # up to ~0.5 s for the local publish to land
        raw = reader.get()
        if raw:
            break
        time.sleep(0.01)
    command = commands_pb2.ProtobufQuestNavCommand()
    command.ParseFromString(raw)
    return command


def _quest_and_reader():
    ntcore.NetworkTableInstance.get_default()
    quest = QuestNav()
    return quest, quest.command_topic.subscribe(k_command_type, b"")


def _send_reset(quest):
    command_id = quest.set_pose(Pose3d(Translation3d(1.0, 2.0, 0.0), Rotation3d()))
    assert command_id is not None
    return command_id


def test_reset_is_not_cleared_before_the_headset_answers():
    quest, reader = _quest_and_reader()
    _send_reset(quest)
    quest.command_periodic()             # same loop - too soon
    assert _request(reader).type == commands_pb2.POSE_RESET


def test_reset_is_cleared_once_the_headset_answers():
    quest, reader = _quest_and_reader()
    command_id = _send_reset(quest)
    quest._command_results[command_id] = True      # what the response topic records
    quest.command_periodic()
    command = _request(reader)
    assert command.type == commands_pb2.COMMAND_TYPE_UNSPECIFIED
    assert command.command_id == QuestNav.k_clear_command_id


def test_reset_is_cleared_after_the_timeout_without_an_answer():
    quest, reader = _quest_and_reader()
    _send_reset(quest)
    quest._clear_pending_since -= QuestNav.k_clear_after_s     # pretend the time has passed
    quest.command_periodic()
    assert _request(reader).type == commands_pb2.COMMAND_TYPE_UNSPECIFIED


def test_a_newer_reset_restarts_the_wait():
    quest, reader = _quest_and_reader()
    first = _send_reset(quest)
    second = _send_reset(quest)
    quest._command_results[first] = True           # an answer to the OLD reset is not enough
    quest.command_periodic()
    assert _request(reader).type == commands_pb2.POSE_RESET
    quest._command_results[second] = True
    quest.command_periodic()
    assert _request(reader).type == commands_pb2.COMMAND_TYPE_UNSPECIFIED
