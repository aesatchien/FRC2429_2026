"""
QuestNav Python Library

Python implementation of questnav-lib for FRC robots.

Usage:
    from questnav import QuestNav, PoseFrame
    
    questnav = QuestNav()
    frames = questnav.get_all_unread_pose_frames()
"""

from dataclasses import dataclass
from typing import List, Optional
import time

import ntcore
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Quaternion

# Import generated protobuf classes
import helpers.questnav.protos.generated.data_pb2 as data_pb2
import helpers.questnav.protos.generated.commands_pb2 as commands_pb2
import helpers.questnav.protos.generated.geometry3d_pb2 as geometry3d_pb2


@dataclass
class PoseFrame:
    """
    Represents a single frame of pose tracking data from the Quest headset.
    
    Mirrors the Java PoseFrame record from questnav-lib.
    
    Attributes:
        quest_pose_3d: The Quest's 3D pose in field coordinates
        data_timestamp: NetworkTables timestamp when data was received (use for pose estimator)
        app_timestamp: Quest app internal timestamp (for debugging only)
        frame_count: Sequential frame number from Quest
    """
    quest_pose_3d: Pose3d
    data_timestamp: float
    app_timestamp: float
    frame_count: int


class QuestNav:
    """
    Python implementation of the Java QuestNav class.
    
    Provides interface for communicating with a Meta Quest VR headset for
    robot localization in FRC robotics applications.
    
    This class handles:
    - Real-time pose tracking data from Quest
    - Device status monitoring (battery, tracking state)
    - Command sending (pose reset)
    - Connection monitoring
    
    Usage:
        # Create instance
        questnav = QuestNav()
        
        # Set initial pose
        from wpimath.geometry import Pose2d, Rotation2d
        initial_pose = Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90))
        questnav.set_pose(Pose3d(initial_pose))
        
        # In robotPeriodic():
        questnav.command_periodic()
        
        frames = questnav.get_all_unread_pose_frames()
        for frame in frames:
            if questnav.is_connected() and questnav.is_tracking():
                # Use frame.quest_pose_3d with pose estimator
                pass
    """

    # --- clearing a stale pose reset off /QuestNav/request --------------------------------
    # NT keeps the LAST value of /QuestNav/request.  If the QuestNav app restarts or resumes
    # (a double-tap into passthrough, the GUI's adb restart), it reads that value again and
    # re-applies the old POSE_RESET - teleporting the headset back to wherever we last synced,
    # even though its own tracking was fine.  So once a reset has been delivered, the request
    # is overwritten with a no-op (COMMAND_TYPE_UNSPECIFIED) and there is nothing to replay.
    #
    # NOT right after sending: NT only delivers the newest value each transmit cycle, so a
    # clear sent too soon can replace the POSE_RESET before the headset ever receives it.
    # The clear waits for the headset's response to that command, or k_clear_after_s.
    # (The first try at this, Apr 2026 feat/quest_clear_cmd, cleared on the first frame after
    # set_pose - possibly before the reset had left the robot.)
    k_clear_command_id = 0xFFFF   # set_pose ids count up from 1 and never get here
    k_clear_after_s = 0.5         # several NT transmit cycles: the reset has been delivered

    def __init__(self):
        """
        Creates a new QuestNav instance.
        
        Initializes NetworkTables subscribers and publishers for communication
        with the Quest headset.
        """
        # Get NetworkTables instance (default instance used by robot)
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        
        # Get QuestNav table
        self.quest_nav_table = self.nt_instance.getTable("QuestNav")
        
        # Use MultiSubscriber to receive all QuestNav topics
        # Include both /QuestNav/ and QuestNav/ to handle different topic naming
        self.multi_sub = ntcore.MultiSubscriber(self.nt_instance, ["/QuestNav/", "QuestNav/"])
        
        # Set up listener for all QuestNav data
        self.data_listener = ntcore.NetworkTableListenerPoller(self.nt_instance)
        self.data_listener.addListener(
            self.multi_sub,
            ntcore.EventFlags.kValueAll
        )
        
        # Publishers for commands (must match Quest's subscriber topic)
        # Quest subscribes to /QuestNav/request as protobuf type
        # We need to publish with the correct protobuf type string
        self.command_topic = self.nt_instance.getRawTopic("/QuestNav/request")
        self.command_pub = self.command_topic.publish("proto:questnav.protos.commands.ProtobufQuestNavCommand")
        
        # State
        self._last_frame_timestamp = 0.0
        self._battery_percent = 0
        self._tracking = False
        self._tracking_lost_counter = 0
        self._frame_count = 0
        self._last_command_id = 0
        self._command_results = {}  # command_id -> success, from the headset's response topic
        self._clear_pending_id = None  # pose reset whose request still needs clearing
        self._clear_pending_since = 0.0
        
        # Queues for unread frames
        self._unread_frames: List[PoseFrame] = []
    
    def get_all_unread_pose_frames(self) -> List[PoseFrame]:
        """
        Retrieves all new pose frames received since the last call.
        
        This is the primary method for integrating QuestNav with FRC pose
        estimation systems. Returns array of PoseFrame objects containing
        pose data and timestamps.
        
        Each frame contains:
        - Pose data: Quest position and orientation in field coordinates
        - NetworkTables timestamp: When data was received (use for pose estimation)
        - App timestamp: Quest internal timestamp (for debugging)
        - Frame count: Sequential frame number
        
        Returns:
            List of PoseFrame objects. Empty list if no new frames available.
        
        Example:
            frames = questnav.get_all_unread_pose_frames()
            for frame in frames:
                if questnav.is_tracking() and questnav.is_connected():
                    pose_estimator.add_vision_measurement(
                        frame.quest_pose_3d.toPose2d(),
                        frame.data_timestamp,
                        (0.1, 0.1, 0.05)  # Standard deviations
                    )
        """
        # Process all new events
        events = self.data_listener.readQueue()
        current_time = time.time()
        
        for event in events:
            try:
                topic_name = event.data.topic.getName()
                value = event.data.value
                # Get timestamp - check which attribute exists
                if hasattr(event.data, 'time'):
                    server_timestamp = event.data.time / 1_000_000.0
                elif hasattr(event.data, 'timestamp'):
                    server_timestamp = event.data.timestamp
                else:
                    server_timestamp = current_time
                
                # Parse frameData
                if "frameData" in topic_name:
                    raw_data = value.getRaw() if hasattr(value, 'getRaw') else bytes()
                    
                    if raw_data:
                        frame_data = data_pb2.ProtobufQuestNavFrameData()
                        frame_data.ParseFromString(raw_data)
                        
                        self._frame_count = frame_data.frame_count
                        self._last_frame_timestamp = current_time
                        self._tracking = frame_data.isTracking # This was changed in ver. 2025.2.2.0 beta
                        
                        # Extract Pose3d
                        pose_proto = frame_data.pose3d
                        trans = pose_proto.translation
                        rot_quat = pose_proto.rotation.q
                        
                        translation = Translation3d(trans.x, trans.y, trans.z)
                        quaternion = Quaternion(rot_quat.w, rot_quat.x, rot_quat.y, rot_quat.z)
                        rotation = Rotation3d(quaternion)
                        pose = Pose3d(translation, rotation)
                        
                        # Create PoseFrame
                        pose_frame = PoseFrame(
                            quest_pose_3d=pose,
                            data_timestamp=server_timestamp,
                            app_timestamp=frame_data.timestamp,
                            frame_count=frame_data.frame_count
                        )
                        
                        self._unread_frames.append(pose_frame)
                
                # Parse deviceData
                elif "deviceData" in topic_name:
                    raw_data = value.getRaw() if hasattr(value, 'getRaw') else bytes()
                    
                    if raw_data:
                        device_data = data_pb2.ProtobufQuestNavDeviceData()
                        device_data.ParseFromString(raw_data)
                        
                        self._battery_percent = device_data.battery_percent
                        # self._tracking = device_data.currently_tracking  # This worked for ver. 2025.2.1.0 beta
                        self._tracking_lost_counter = device_data.tracking_lost_counter
                
                # Parse command responses
                elif "response" in topic_name:
                    raw_data = value.getRaw() if hasattr(value, 'getRaw') else bytes()
                    
                    if raw_data:
                        response = commands_pb2.ProtobufQuestNavCommandResponse()
                        response.ParseFromString(raw_data)

                        if response.command_id == self.k_clear_command_id:
                            continue  # our own no-op (see clear_command) - the app may call it a failure
                        self._command_results[response.command_id] = response.success
                        if len(self._command_results) > 16:  # keep it small, only recent ids matter
                            self._command_results.pop(next(iter(self._command_results)))
                        if not response.success:
                            print(f"QuestNav command {response.command_id} failed: {response.error_message}")
                        
            except Exception as e:
                print(f"QuestNav error processing data: {e}")
        
        # Return all unread frames and clear queue
        frames = self._unread_frames.copy()
        self._unread_frames.clear()
        return frames
    
    def get_command_result(self, command_id):
        """Headset's answer to a command: True/False once a response has arrived, else None."""
        return self._command_results.get(command_id)

    def set_pose(self, pose: Pose3d):
        """
        Sets the field-relative pose of the Quest headset.
        
        Sends a pose reset command to the Quest, telling it where it is
        currently located on the field. Essential for establishing field-relative
        tracking.
        
        Call this:
        - At start of autonomous/teleop when Quest position is known
        - When robot is placed at a known location
        - After significant tracking drift
        
        Important: This should be the Quest's pose, not the robot's pose.
        If you know the robot's pose, apply the mounting offset to get Quest pose.
        
        Args:
            pose: The Quest's current field-relative pose in WPILib coordinates
        
        Example:
            # If you know Quest pose directly
            quest_pose = Pose3d(1.5, 5.5, 0.0, Rotation3d())
            questnav.set_pose(quest_pose)
            
            # If you know robot pose, apply mounting offset
            robot_pose = pose_estimator.getEstimatedPosition()
            quest_pose = Pose3d(robot_pose).transformBy(mounting_offset)
            questnav.set_pose(quest_pose)
        """
        self._last_command_id += 1
        
        try:
            # Create command protobuf
            command = commands_pb2.ProtobufQuestNavCommand()
            command.type = commands_pb2.POSE_RESET
            command.command_id = self._last_command_id
            
            # Create pose reset payload
            payload = commands_pb2.ProtobufQuestNavPoseResetPayload()
            
            # Set target pose
            pose_proto = geometry3d_pb2.ProtobufPose3d()
            pose_proto.translation.x = pose.translation().X()
            pose_proto.translation.y = pose.translation().Y()
            pose_proto.translation.z = pose.translation().Z()
            
            quat = pose.rotation().getQuaternion()
            pose_proto.rotation.q.w = quat.W()
            pose_proto.rotation.q.x = quat.X()
            pose_proto.rotation.q.y = quat.Y()
            pose_proto.rotation.q.z = quat.Z()
            
            payload.target_pose.CopyFrom(pose_proto)
            command.pose_reset_payload.CopyFrom(payload)
            
            # Publish command
            serialized = command.SerializeToString()
            self.command_pub.set(serialized)
            self._clear_pending_id = self._last_command_id  # see command_periodic
            self._clear_pending_since = time.monotonic()
            return self._last_command_id

        except Exception as e:
            print(f"QuestNav error sending pose reset: {e}")
            return None  # nothing was sent
    
    def get_battery_percent(self) -> int:  # was Optional[int]:
        """
        Returns the Quest headset's current battery level as a percentage.
        
        Returns:
            Battery percentage (0-100), or None if no data available
        """
        return self._battery_percent if self._battery_percent > 0 else 0  # was None
    
    def is_tracking(self) -> bool:
        """
        Gets the current tracking state of the Quest headset.
        
        Indicates whether the Quest's visual-inertial tracking system is
        currently functioning and providing reliable pose data.
        
        When tracking is lost, pose data becomes unreliable and should not
        be used for robot control.
        
        Returns:
            True if Quest is actively tracking, False if tracking is lost
            or no device data available
        """
        return self._tracking
    
    def is_connected(self) -> bool:
        """
        Determines if the Quest headset is currently connected.
        
        Connection is determined by how recent the last frame data was received.
        
        Returns:
            True if Quest is connected and sending data, False otherwise
        """
        current_time = time.time()
        return (current_time - self._last_frame_timestamp) < 0.2  # 100ms timeout, relaxed to 200ms
    
    def get_frame_count(self) -> int:  #was Optional[int]:
        """
        Gets the current frame count from the Quest headset.
        
        Returns:
            Frame count value, or None if no data available
        """
        return self._frame_count if self._frame_count > 0 else 0  # was None
    
    def get_tracking_lost_counter(self) -> int: # was Optional[int]:
        """
        Gets the number of tracking lost events since Quest connected.
        
        Returns:
            Tracking lost counter value, or None if no data available
        """
        return self._tracking_lost_counter
    
    def get_latency(self) -> float:
        """
        Gets the latency of the Quest to Robot connection.
        
        Returns latency between current time and last frame data update.
        
        Returns:
            Latency in milliseconds
        """
        current_time = time.time()
        return (current_time - self._last_frame_timestamp) * 1000.0
    
    def get_app_timestamp(self) -> float: # was Optional[float]:
        """
        Returns the Quest app's uptime timestamp.
        
        Important: For pose estimator integration, use the timestamp from
        PoseFrame.data_timestamp instead! This provides the Quest's internal
        timestamp for debugging only.
        
        Returns:
            Quest app uptime in seconds, or None if no data available
        """
        # This would need to be tracked from frame data
        # For now, return None
        return 0 # was None
    
    def command_periodic(self):
        """
        Processes command responses from the Quest headset.
        
        Must be called regularly (typically in robotPeriodic()) to:
        - Process responses to commands sent via set_pose()
        - Log command failures for debugging
        - Maintain proper command/response synchronization
        
        Call this every robot loop (20ms).
        
        Example:
            def robotPeriodic(self):
                self.questnav.command_periodic()
                # ... other code
        """
        # Command responses are processed in get_all_unread_pose_frames().  The one job here:
        # clear a delivered pose reset off /QuestNav/request (see k_clear_command_id above).
        if self._clear_pending_id is None:
            return
        answered = self._command_results.get(self._clear_pending_id) is not None
        if answered or time.monotonic() - self._clear_pending_since >= self.k_clear_after_s:
            self.clear_command()

    def clear_command(self):
        """Overwrite /QuestNav/request with a no-op so a restarted or resumed QuestNav app has
        no old POSE_RESET to re-apply.  Called by command_periodic once a reset is delivered."""
        self._clear_pending_id = None
        try:
            command = commands_pb2.ProtobufQuestNavCommand()
            command.type = commands_pb2.COMMAND_TYPE_UNSPECIFIED
            command.command_id = self.k_clear_command_id
            self.command_pub.set(command.SerializeToString())
        except Exception as e:
            print(f"QuestNav error clearing the pose reset request: {e}")


__all__ = ['QuestNav', 'PoseFrame']
__version__ = '2025.1.0'

