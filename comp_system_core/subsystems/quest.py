import wpilib
from helpers.nt_time import nt_to_seconds
import random
from commands2 import Subsystem, InstantCommand
from wpilib import Alliance, MatchState, Timer
from helpers.dashboard import SmartDashboard  # 2027a7: wpilib's was removed
from wpimath import Pose2d, Rotation2d, Translation2d, Pose3d, Transform2d
from ntcore import NetworkTableInstance

from helpers.questnav.questnav import QuestNav as Metaquestnav
from helpers.quest_double_tap import DoubleTapWatch

import constants
from constants import FieldConstants as fc, QuestConstants as qc

# TODO - clean and speed this up
class Questnav(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.set_name('Quest')
        self.counter = constants.QuestConstants.k_counter_offset  # increments in periodic

        self.questnav = Metaquestnav()  # create the QuestNav instance

        self.quest_pose_accepted = False  # validate our location on a field

        self.quest_to_robot = constants.QuestConstants.quest_to_robot  # 10.50 -8.35
        # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        # self.quest_field = Field2d()

        # ping the quest - it seems to respond to is_connected only after you get frames ~ 2s after we connect
        # and although the headset says it is connected to the sim, is_connected() stays False unless you get frames
        self._ping_connection()

        self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
        self.use_quest = constants.k_use_quest_odometry  # do we allow the quest to update odometry

        if wpilib.RobotBase.is_real():
            self.quest_pose = Pose2d(-10, -10, Rotation2d.from_degrees(0)) # initial pose if not connected / tracking
        else:
            self.quest_pose = Pose2d(2, 6, Rotation2d.from_degrees(180))
        self.quest_pose_timestamp = 0.0

        # --- WATCHDOGS (CRITICAL ORDERING) ---
        # 1. missed_frame_count (qc.k_max_missed_frames): the pose STREAM is lost.  Remembers
        #    where we were for the soft resync and, if double taps are possible, asks the
        #    dashboard for the adb fix (helpers/quest_double_tap.py).
        #    Must be >= 10 loops to absorb normal 100ms NT4 network batching and wake-up stutters.
        # 2. disconnected_count (qc.k_max_disconnected_count): the hard odometry unsync.
        #    Because disconnected_count only starts ticking AFTER the 200ms network timeout,
        #    whatever is bringing the stream back gets a head start before we wipe the field sync.
        self.missed_frame_count = 0
        self.k_max_missed_frames = qc.k_max_missed_frames
        self.was_tracking = False
        self.was_connected = False
        self.disconnected_count = 0  # count how many frames we've been disconnected to avoid ping-ponging in and out of sync
        self.out_of_bounds_count = 0
        self.k_max_disconnected_count = qc.k_max_disconnected_count  # lost iterations before we say we are disconnected
        # A pose reset we sent but the headset has not proven it applied yet.  Until a frame
        # lands near the target (or it fails / times out) the pose is NOT accepted, because
        # the frames right after a reset can still carry the pre-reset pose.
        self.pending_reset_id = None  # command id from questnav.set_pose()
        self.pending_reset_pose = None  # robot-frame pose we asked for
        self.pending_reset_start = 0.0
        self.pending_reset_marks_synced = False  # True for a sync, False for a plain reset
        self.k_reset_timeout = 1.0  # s to wait for the headset to show the new pose
        self.k_reset_tolerance_m = 0.5  # how close a frame must be to the target to count as applied
        self.k_reset_tolerance_deg = 30.0
        self.error_pose = Pose2d(0,0,0)  # difference between robot and quest
        # what we knew when the pose stream was lost, for the soft resync when it comes back
        self.outage_start_time = 0.0
        self.synced_before_outage = False
        self.pre_outage_pose = Pose2d()

        # the double tap silo - a no-op unless qc.k_check_for_double_tap
        self.double_tap = DoubleTapWatch(qc.k_check_for_double_tap)

        # In sim the Quest is either the mock (made up from ground truth) or a real headset on
        # the network - SimConstants.k_quest_source.  On the robot it is always the real one.
        sim = constants.SimConstants
        self.mock_questnav = wpilib.RobotBase.is_simulation() and sim.k_quest_source == 'sim'

        # The mock's error: where it thinks it is, relative to the truth.  Starts off badly
        # wrong so that syncing visibly does something, then drifts by a random walk
        # (uniform steps, so sigma per step ~ a/sqrt(3) and after n steps ~ sqrt(n) * sigma).
        x, y, deg = sim.k_mock_quest_start_error
        self.sim_offset_from_truth = Transform2d(Translation2d(x, y), Rotation2d.from_degrees(deg))
        self.walk_xy = sim.k_mock_quest_walk_xy  # meters per loop
        self.walk_deg = sim.k_mock_quest_walk_deg  # degrees per loop

        self._init_networktables()

    def _ping_connection(self):
        connected_before = self.questnav.is_connected()
        self.questnav.get_all_unread_pose_frames()  # reading the frames is what makes the quest ACK
        connected_after = self.questnav.is_connected()
        if connected_after != connected_before:
            # let us know if the situation changes
            print(f'*** {Timer.get_timestamp():04.1f}s: Questnav connection status {connected_before} before ping and {connected_after} after  ***')

    def _init_networktables(self):
        self.inst = NetworkTableInstance.get_default()
        quest_prefix = constants.quest_prefix
        swerve_prefix = constants.swerve_prefix
        sim_prefix = constants.sim_prefix

        # ------------- Publishers (Efficiency) -------------
        self.quest_synched_pub = self.inst.get_boolean_topic(f"{quest_prefix}/questnav_synched").publish()
        self.quest_in_use_pub = self.inst.get_boolean_topic(f"{quest_prefix}/questnav_in_use").publish()
        
        self.quest_accepted_pub = self.inst.get_boolean_topic(f"{quest_prefix}/quest_pose_accepted").publish()
        self.quest_connected_pub = self.inst.get_boolean_topic(f"{quest_prefix}/quest_connected").publish()
        self.quest_tracking_pub = self.inst.get_boolean_topic(f"{quest_prefix}/quest_tracking").publish()
        
        # Use StructPublisher for Pose2d - matches Swerve implementation and works with AdvantageScope
        self.quest_pose_pub = self.inst.get_struct_topic(f"{quest_prefix}/quest_pose", Pose2d).publish()
        self.quest_error_pose_pub = self.inst.get_struct_topic(f"{quest_prefix}/quest_error_pose", Pose2d).publish()
        # self.pose_pub = self.inst.getDoubleArrayTopic(f"{quest_prefix}/QUEST_POSE").publish()  # legacy GUI dashboard
        
        self.quest_battery_pub = self.inst.get_double_topic(f"{quest_prefix}/quest_Battery_pct").publish()
        self.quest_latency_pub = self.inst.get_double_topic(f"{quest_prefix}/quest_Latency").publish()
        self.quest_lost_count_pub = self.inst.get_double_topic(f"{quest_prefix}/quest_tracking_lost_count").publish()
        self.quest_frame_count_pub = self.inst.get_double_topic(f"{quest_prefix}/quest_frame_count").publish()
        
        # quest_in_passthrough / quest_dtap_count / check_for_double_tap: helpers/quest_double_tap.py

        # ------------- Subscribers -------------
        # Subscribe to the drive_pose published by Swerve (now a Struct)
        self.drive_pose_sub = self.inst.get_struct_topic(f"{swerve_prefix}/drive_pose", Pose2d).subscribe(Pose2d())

        # Subscribe to ground truth from Physics (for Sim) - doesn't really matter if it doesn't exist when real
        self.ground_truth_sub = self.inst.get_struct_topic(f"{sim_prefix}/ground_truth", Pose2d).subscribe(Pose2d())

        # ------------- Initial Values & Buttons -------------
        self.quest_synched_pub.set(self.quest_has_synched)
        self.quest_in_use_pub.set(self.use_quest)

        # note - may not want these buried one deeper.  Also, we may want to put them in robotcontainer instead of here
        command_prefix = constants.command_prefix
        SmartDashboard.put_data(f'{command_prefix}/QuestResetOdometry', InstantCommand(lambda: self.quest_reset_odometry()).ignoring_disable(True))
        SmartDashboard.put_data(f'{command_prefix}/QuestSyncOdometry', InstantCommand(lambda: self.quest_sync_odometry()).ignoring_disable(True))
        SmartDashboard.put_data(f'{command_prefix}/QuestEnableToggle', InstantCommand(lambda: self.quest_enabled_toggle()).ignoring_disable(True))
        SmartDashboard.put_data(f'{command_prefix}/QuestSyncToggle', InstantCommand(lambda: self.quest_sync_toggle()).ignoring_disable(True))
        
        # Put Field2d once (*** it updates itself internally ***)
        # TODO - this needs to be broadcast based on the prefixes in constants
        # SmartDashboard.putData("Quest/Field", self.quest_field)

    def set_quest_pose(self, pose: Pose2d) -> None:
        # set the pose of the Questnav, transforming from robot center top questnav coordinate
        command_id = self.questnav.set_pose(Pose3d(pose.transform_by(self.quest_to_robot.inverse())))
        self.pending_reset_id = command_id
        self.pending_reset_pose = pose
        self.pending_reset_start = Timer.get_timestamp()
        if command_id is None:  # nothing was sent
            self._fail_pending_reset("command could not be sent")

        if self.mock_questnav:
            # In Sim, calculate the error needed so that (Truth + Error) = TargetPose
            # This allows us to "reset" the quest to a specific field location even if the robot isn't there
            ground_truth = self.ground_truth_sub.get()
            self.sim_offset_from_truth = Transform2d(
                pose.x - ground_truth.x,
                pose.y - ground_truth.y,
                pose.rotation() - ground_truth.rotation()
            )
            print(f"Sim Quest Reset: Error reset to {self.sim_offset_from_truth.x:.2f}, {self.sim_offset_from_truth.y:.2f}")
            self._finish_pending_reset()  # no headset to wait for, the mock applies it instantly

    def _finish_pending_reset(self) -> None:
        """The headset showed the pose we asked for - now it is safe to call it synced."""
        marks_synced = self.pending_reset_marks_synced
        self.pending_reset_id = None
        self.pending_reset_pose = None
        self.pending_reset_marks_synced = False
        if marks_synced:
            self.quest_has_synched = True  # let the robot know we have been synched so we don't automatically do it again
            self.quest_synched_pub.set(self.quest_has_synched)

    def _fail_pending_reset(self, reason: str) -> None:
        print(f"*** QuestNav pose reset not applied ({reason}) at {Timer.get_timestamp():.2f}s ***")
        self.pending_reset_id = None
        self.pending_reset_pose = None
        self.pending_reset_marks_synced = False
        self.quest_unsync_odometry()  # never leave a failed reset looking synced

    def _check_pending_reset(self, frame_pose) -> None:
        """Called every loop while a reset is pending; frame_pose is the newest robot-frame quest pose or None."""
        if self.mock_questnav:
            return
        if self.questnav.get_command_result(self.pending_reset_id) is False:
            self._fail_pending_reset("headset reported failure")
        elif (frame_pose is not None
              and frame_pose.translation().distance(self.pending_reset_pose.translation()) < self.k_reset_tolerance_m
              and abs((frame_pose.rotation() - self.pending_reset_pose.rotation()).degrees()) < self.k_reset_tolerance_deg):
            self._finish_pending_reset()
        elif Timer.get_timestamp() - self.pending_reset_start > self.k_reset_timeout:
            self._fail_pending_reset("headset never showed the new pose")

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        # this came over from swerve, maybe we don't need it anymore
        #self.reset_pose(pose)
        self.set_quest_pose(pose)

    def quest_reset_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        if not self.mock_questnav and not self.questnav.is_connected():
            print(f"*** Cannot reset QuestNav: Headset not connected at {Timer.get_timestamp():.1f}s ***")
            return

        red_pose = Pose2d(14.0, 4.00, Rotation2d.from_degrees(0))
        blue_pose = Pose2d(3., 4.00, Rotation2d.from_degrees(180))

        if MatchState.get_alliance() == Alliance.RED:
            new_pose = red_pose
        else:
            new_pose = blue_pose

        self.pending_reset_marks_synced = False  # a reset supersedes any sync still in flight
        self.set_quest_pose(new_pose)
        print(f"Reset questnav at {Timer.get_timestamp():.2f}s")
        self.quest_unsync_odometry()

    def quest_sync_odometry(self) -> None:
        if not self.mock_questnav and not self.questnav.is_connected():
            print(f"*** Cannot sync QuestNav: Headset not connected at {Timer.get_timestamp():.2f}s ***")
            return

        if self.pending_reset_id is not None:
            return  # a reset is already in flight - the auto-sync callers retry, so don't pile up commands

        # quest_has_synched is only set once the headset has shown the new pose (see
        # _check_pending_reset) - sending the command is not the same as it being applied
        self.pending_reset_marks_synced = True

        if self.mock_questnav:
            # In Sim, "Sync" means agree with Ground Truth (remove all drift/error)
            self.set_quest_pose(self.ground_truth_sub.get())
        else:
            # In Real life, "Sync" means agree with the Robot's Odometry
            # Although, really, the swerve sim is basically serving that same sim ground truth

            print("Sync command sent to questnav, waiting for the headset to apply it")
            self.set_quest_pose(self.drive_pose_sub.get())

        print(f'  syncing quest to {self.drive_pose_sub.get()} (quest was at {self.quest_pose})')

    def quest_soft_resync(self):
        # allow quest to resync to itself without changing the pose
        # handle the case where we disconnect but the pose is still accurate when we come back up
        self.quest_has_synched = True
        self.quest_synched_pub.set(self.quest_has_synched)
        print(f'  Soft-resynced quest at {Timer.get_timestamp():.2f}s')

    def quest_unsync_odometry(self) -> None:
        self.quest_has_synched = False  # let the robot know we have been synched so we don't automatically do it again
        print(f'  Unsynched quest at {Timer.get_timestamp():.2f}s')
        self.quest_synched_pub.set(self.quest_has_synched)

    def quest_enabled_toggle(self, force=None):  # allow us to stop using quest if it is a problem - 20251014 CJH
        if force is None:
            self.use_quest = not self.use_quest  # toggle the boolean
        elif force == 'on':
            self.use_quest = True
        elif force == 'off':
            self.use_quest = False
        else:
            self.use_quest = False

        print(f'swerve use_quest updated to {self.use_quest} at {Timer.get_timestamp():.1f}s')
        self.quest_in_use_pub.set(self.use_quest)

    def quest_sync_toggle(self, force=None):  # toggle sync state for dashboard - 20251014 CJH
        current_state = self.quest_has_synched
        if force is None:
            current_state = not current_state  # toggle the boolean
        elif force == 'on':
            current_state = True
        elif force == 'off':
            current_state = False
        else:
            current_state = False

        if current_state:
            self.quest_sync_odometry()
        else:
            self.quest_unsync_odometry()
        # reporting done by the sync/unsync functions

    def is_quest_enabled(self):
        return self.use_quest

    def is_pose_accepted(self):
        return self.quest_pose_accepted

    def is_quest_connected(self):
        # the mock has no headset to be connected to, so it is always "connected" - otherwise
        # Swerve's fusion gate would never open in mock sim
        return True if self.mock_questnav else self.questnav.is_connected()

    def periodic(self) -> None:
        self.counter += 1

        self.questnav.command_periodic()

        frames, is_connected, is_tracking = self._read_headset()
        self._watch_connection(is_connected)

        if self.mock_questnav:
            self._update_pose_from_mock()
        else:  # real robot, or sim with a real headset (hardware-in-the-loop)
            self._update_pose_from_frames(frames, is_tracking)

        if self.pending_reset_id is not None:
            newest = self.quest_pose if (is_tracking and frames and not self.mock_questnav) else None
            self._check_pending_reset(newest)

        # calculate pose error
        self.error_pose = self.quest_pose.relative_to(self.drive_pose_sub.get())

        # NOT throttled: Swerve needs the cutoff the moment the data goes bad
        self._update_acceptance(is_connected, is_tracking)

        if self.counter % 5 == 0:
            self._publish(is_connected, is_tracking)

        # ping the quest every few seconds in sim to check to get a connection to physical hardware
        if wpilib.RobotBase.is_simulation() and not self.mock_questnav and self.counter % 200 == 0 and not is_connected:
            self._ping_connection()

    # ------------------------------------------------------------------------------------
    # periodic(), one step at a time
    # ------------------------------------------------------------------------------------

    def _read_headset(self):
        """-> (frames, is_connected, is_tracking).

        Read the frames FIRST: reading is what updates the headset's connected/tracking state,
        so caching the flags before it acted on the previous loop's state.  The mock has no
        headset, so it is always connected and tracking."""
        if self.mock_questnav:
            return [], True, True
        frames = self.questnav.get_all_unread_pose_frames()
        return frames, self.questnav.is_connected(), self.questnav.is_tracking()

    def _watch_connection(self, is_connected: bool) -> None:
        """Connection watchdog: gone for longer than k_max_disconnected_count loops -> unsync."""
        if is_connected:
            self.disconnected_count = 0
            self.was_connected = True
            return
        self.disconnected_count += 1
        if self.disconnected_count > self.k_max_disconnected_count and self.was_connected:
            print(f"*** QuestNav connection dropped for  {self.disconnected_count / 50:.2f}s at {wpilib.Timer.get_timestamp():.2f}s. ***")
            self.quest_unsync_odometry()
            self.was_connected = False

    def _update_pose_from_frames(self, frames, is_tracking: bool) -> None:
        """Stream watchdog: take the newest frame, and notice the stream stopping and restarting."""
        # Only process if actively tracking AND frames exist.
        if is_tracking and frames:
            self.missed_frame_count = 0  # Reset watchdog
            frame_last = frames[-1]  # only read the last frame if multiple are available
            try:
                self.quest_pose = frame_last.quest_pose_3d.to_pose2d().transform_by(self.quest_to_robot)
                self.quest_pose_timestamp = frame_last.data_timestamp
                if not self.was_tracking:
                    self._on_stream_restored()
                self.was_tracking = True  # Successfully processed a valid tracking frame
            except Exception as e:
                print(f"Error processing Quest frame: {e}")
                self.was_tracking = False  # Actively flag blackout state on exception
        else:
            # no usable frame this loop: app suspended, tracking lost, wifi, headset off ...
            self.missed_frame_count += 1
            if self.missed_frame_count > self.k_max_missed_frames and self.was_tracking:
                self.was_tracking = False
                self._on_stream_lost()

    def _on_stream_lost(self) -> None:
        self.outage_start_time = wpilib.Timer.get_timestamp()
        self.synced_before_outage = self.quest_has_synched
        self.pre_outage_pose = self.quest_pose
        self.double_tap.on_stream_lost()
        print(f"Detecting a lost QuestNav at FPGA timestamp: {wpilib.Timer.get_timestamp():.2f}s")

    def _on_stream_restored(self) -> None:
        """First good frame after an outage (self.quest_pose is already that frame)."""
        print(f"  Quest tracking restored at {wpilib.Timer.get_timestamp():.2f}s ... ")
        self.double_tap.on_stream_restored()

        # Soft resync: a short outage that cost us the sync, where the headset comes back about
        # where it was, kept its own tracking the whole time - so the old sync is still good.
        outage_time = wpilib.Timer.get_timestamp() - self.outage_start_time
        if outage_time < qc.k_max_resync_time and self.synced_before_outage and not self.quest_has_synched:
            distance_moved = self.quest_pose.translation().distance(self.pre_outage_pose.translation())
            if distance_moved < qc.k_max_outage_distance:
                print(f"  Quest recovered in {outage_time:.2f}s. Soft resyncing. Robot pose delta of {self.error_pose.x:.2f}, {self.error_pose.y:.2f}, {self.error_pose.rotation().degrees():.1f}°.")
                self.quest_soft_resync()
            else:
                print(f"  Quest recovered but moved too far ({distance_moved:.2f}m > {qc.k_max_outage_distance}m). Skipping soft resync.")
                print(f"    Old Pose: X={self.pre_outage_pose.x:.2f}, Y={self.pre_outage_pose.y:.2f} | New Pose: X={self.quest_pose.x:.2f}, Y={self.quest_pose.y:.2f}")

    def _update_pose_from_mock(self) -> None:
        """Sim without a headset: ground truth from the physics sim plus the mock's drift/error."""
        # Add a random walk to the error to simulate drift
        self.sim_offset_from_truth = Transform2d(
            self.sim_offset_from_truth.x + (random.random() - 0.5) * 2 * self.walk_xy,
            self.sim_offset_from_truth.y + (random.random() - 0.5) * 2 * self.walk_xy,
            Rotation2d.from_degrees(
                self.sim_offset_from_truth.rotation().degrees() + (random.random() - 0.5) * 2 * self.walk_deg)
        )

        ground_truth_atomic = self.ground_truth_sub.get_atomic()
        ground_truth: Pose2d = ground_truth_atomic.value
        # Apply the offset field-relatively (simple addition), not robot-relatively (transform_by)
        self.quest_pose = Pose2d(
            ground_truth.x + self.sim_offset_from_truth.x,
            ground_truth.y + self.sim_offset_from_truth.y,
            ground_truth.rotation() + self.sim_offset_from_truth.rotation()
        )
        self.quest_pose_timestamp = nt_to_seconds(ground_truth_atomic.time)  # NT seconds, see helpers/nt_time.py
        self.was_tracking = True  # Sim never loses tracking

    def _update_acceptance(self, is_connected: bool, is_tracking: bool) -> None:
        # If we missed >5 frames (100ms), we missed an NT heartbeat. Suspend immediately.
        data_is_fresh = self.missed_frame_count <= 5
        in_bounds = 0 < self.quest_pose.x < fc.k_field_length and 0 < self.quest_pose.y < fc.k_field_width

        # Out of bounds safety check: If the Quest thinks it's outside the field for 0.5s while synced, unsync it.
        if self.quest_has_synched and not in_bounds:
            self.out_of_bounds_count += 1
            if self.out_of_bounds_count >= 25:
                print(f"*** QuestNav pose out of bounds for >0.5s at {wpilib.Timer.get_timestamp():.2f}s. Forcing unsync. ***")
                self.quest_unsync_odometry()
        else:
            self.out_of_bounds_count = 0

        # --- STRICT POSE ACCEPTANCE CRITERIA ---
        # To feed odometry to the Swerve drive, all of the following must be true:
        # 1. in_bounds: The Quest pose is physically inside the field boundaries.
        # 2. is_connected: NetworkTables heartbeat < 200ms. Survives normal Wi-Fi jitter.
        # 3. is_tracking: The headset's internal SLAM algorithms are confident.
        # 4. self.was_tracking: We didn't just throw an exception on the last frame.
        # 5. data_is_fresh: We unpacked a new frame in the last 5 loops (100ms).
        #    *Why both 2 & 5?* When the pose stream stops (the app suspended by a double tap,
        #    or anything else), `is_connected` stays True for 200ms, but `data_is_fresh` fails
        #    at 100ms, killing odometry updates twice as fast to prevent pose drift.
        # 6. no reset pending - frames right after a reset can still carry the old pose
        self.quest_pose_accepted = (in_bounds and is_connected and is_tracking and self.was_tracking
                                    and data_is_fresh and self.pending_reset_id is None)

    def _publish(self, is_connected: bool, is_tracking: bool) -> None:
        # poses used in gui and advantagescope
        self.quest_pose_pub.set(self.quest_pose)
        self.quest_error_pose_pub.set(self.error_pose)  # the deltas between robot and quest
        self.quest_accepted_pub.set(self.quest_pose_accepted)  # GUI uses as VALID
        self.quest_connected_pub.set(is_connected)  # GUI uses as heartbeat
        self.quest_tracking_pub.set(is_tracking)  # GUI follows to see if tracking

        self.quest_battery_pub.set(self.questnav.get_battery_percent())
        self.quest_latency_pub.set(self.questnav.get_latency())
        self.quest_lost_count_pub.set(self.questnav.get_tracking_lost_counter())
        self.quest_frame_count_pub.set(self.questnav.get_frame_count())
