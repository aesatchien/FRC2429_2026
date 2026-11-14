"""Everything the robot does ONLY because a double tap can suspend the QuestNav app.

WHY THIS EXISTS
---------------
A double tap on the side of the headset (a bump does it too) drops the Quest into passthrough
and suspends the QuestNav app: the pose stream stops although the headset is still tracking.
The cure is an adb `am start` that brings the app back to the front, and that is run by the
dashboard (gui/quest_double_tap.py), not by the robot.  The robot's part is small:

  * raise /QuestNav/quest_in_passthrough when the pose stream stops - the dashboard's cue to
    run adb, and what the LEDs show as DTAP
  * count how many times that happened (/QuestNav/quest_dtap_count)
  * drop the flag when the stream is back

A QuestNav update lets the double tap be disabled on the headset.  With it disabled a lost
stream is just a lost stream (wifi, tracking, a dead headset) and there is nothing for adb to
fix, so constants.QuestConstants.k_check_for_double_tap = False turns all of the above off.

Losing the stream, going unsynced, the soft resync and the teleop auto-resync are NOT double
tap handling - they live in subsystems/quest.py and robot.py and run either way.

The switch is published as /QuestNav/check_for_double_tap so the dashboard follows the robot's
setting instead of keeping one of its own that could disagree.
"""

from ntcore import NetworkTableInstance

import constants


class DoubleTapWatch:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.count = 0  # stream losses we treated as a double tap

        inst = NetworkTableInstance.get_default()
        prefix = constants.quest_prefix
        self.enabled_pub = inst.get_boolean_topic(f"{prefix}/check_for_double_tap").publish()
        self.in_passthrough_pub = inst.get_boolean_topic(f"{prefix}/quest_in_passthrough").publish()
        self.count_pub = inst.get_double_topic(f"{prefix}/quest_dtap_count").publish()

        self.enabled_pub.set(self.enabled)
        self.in_passthrough_pub.set(False)
        self.count_pub.set(self.count)

    def on_stream_lost(self) -> None:
        """The pose stream stopped while we were tracking - ask the dashboard for the adb fix."""
        if not self.enabled:
            return
        self.count += 1
        self.in_passthrough_pub.set(True)
        self.count_pub.set(self.count)

    def on_stream_restored(self) -> None:
        self.in_passthrough_pub.set(False)
