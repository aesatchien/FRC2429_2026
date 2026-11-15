"""
The "virtual coprocessor": works out what each camera would see from where the robot REALLY
is, and publishes it on the same /Cameras/... topics the real Raspberry Pis use.  vision.py
subscribes to those and cannot tell the difference - which is the whole point.

Owned by Vision.simulation_periodic().  Takes ground truth (not the pose estimate - a camera
sees where the robot is, not where it thinks it is) and reads the game pieces back off the
shared Field2d's "Gamepieces" object, so it depends on nothing but the field.
"""

import math
import wpilib
import ntcore
from ntcore import PubSubOptions
from wpimath import Pose2d, Rotation2d, Translation2d
import constants
from helpers import apriltag_utils, dashboard
from simulation.gamepiece_sim import GamepieceSim

class VisionSim:
    def __init__(self):
        self.inst = ntcore.NetworkTableInstance.get_default()
        
        # Configuration
        self.source = constants.SimConstants.k_camera_source  # 'sim', 'real' or 'blink_test'
        self.cam_list = list(constants.CameraConstants.k_cameras.keys())
        self.physical_cameras = sorted(list(set(c['topic_name'] for c in constants.CameraConstants.k_cameras.values())))

        self.camera_dict = {}
        self._init_networktables()
        self._init_field_objects()
        self._init_apriltags()

    def _init_networktables(self):
        sim_prefix = constants.sim_prefix
        
        # FOV Show/Hide Subscribers
        self.show_fov_subs = {
            key: self.inst.get_boolean_topic(f"{sim_prefix}/FOV/{key}_show_fov").subscribe(False)
            for key in self.cam_list
        }

        # Real coprocessors own the /Cameras/... topics: make no publishers at all, so there
        # is nothing of ours on those topics.  camera_dict stays empty and update() only draws FOVs.
        if self.source == 'real':
            return

        # A coprocessor turns "1.9 m away, 4 degrees left" into a field position with the pose
        # the ROBOT reports, not the true one - it has no other.  So do we.
        self.robot_estimate_sub = self.inst.get_struct_topic(f"{constants.swerve_prefix}/drive_pose", Pose2d).subscribe(Pose2d())

        for ix, (key, config) in enumerate(constants.CameraConstants.k_cameras.items()):
            if config.get('external'):
                continue  # published from outside the robot program (see CameraConstants) - not ours
            cam_topic = config['topic_name']
            cam_type = config.get('label', config['type'])
            base = f'/Cameras/{cam_topic}/{cam_type}'
            
            self.camera_dict[key] = {
                'offset': ix,
                'frames': 0,
                'frames_pub': self.inst.get_integer_topic(f"/Cameras/{cam_topic}/_frames").publish(),
                'targets_pub': self.inst.get_double_topic(f"{base}/targets").publish(PubSubOptions(keep_duplicates=True)),  # otherwise targets get stale
                'distance_pub': self.inst.get_double_topic(f"{base}/distance").publish(),
                'strafe_pub': self.inst.get_double_topic(f"{base}/strafe").publish(),
                'rotation_pub': self.inst.get_double_topic(f"{base}/rotation").publish()
            }
            if config['type'] == 'hsv':
                # EVERY target in view, in field coordinates - what a detector that reports
                # more than the nearest one publishes (see Vision.get_field_targets)
                self.camera_dict[key]['field_positions_pub'] = self.inst.get_struct_array_topic(
                    f"{base}/field_positions", Translation2d).publish(PubSubOptions(keep_duplicates=True))
            if config['type'] == 'tags':
                # what a real tag camera sends for odometry: [tag id, x, y, z, rx, ry, rz] of the ROBOT
                self.camera_dict[key]['pose_pub'] = self.inst.get_double_array_topic(
                    f"/Cameras/{cam_topic}/poses/tag1").publish(PubSubOptions(keep_duplicates=True))

    def _init_field_objects(self):
        # Pre-fetch Field2d objects for FOV visualization
        self.fov_objects = {}
        for idx, key in enumerate(self.cam_list):
            self.fov_objects[key] = dashboard.field_object(f"FOV_{idx}")

    def _init_apriltags(self):
        self.tag_translations = []
        self.tag_poses = []
        self.tag_ids = []
        
        # Load tags from the layout utility
        for tag in apriltag_utils.layout.get_tags():
            pose3d = apriltag_utils.layout.get_tag_pose(tag.ID)
            if pose3d is not None:
                pose2d = pose3d.to_pose2d()
                self.tag_poses.append(pose2d)
                self.tag_ids.append(tag.ID)
                self.tag_translations.append(pose2d.translation())
        
        dashboard.field_object("AprilTags").set_poses(self.tag_poses)


    def update(self, robot_pose: Pose2d):
        now = wpilib.Timer.get_timestamp()
        # whatever is still on the floor, as drawn by RobotState's gamepiece sim
        gamepieces = [{'pos': pos, 'active': True} for pos in GamepieceSim.active_positions_from_field()]

        if self.source == 'blink_test':
            self._run_blink_test(now)
            return

        # 'sim' publishes what each camera would see; 'real' has no publishers (camera_dict is
        # empty), so for it this loop only draws the FOVs
        for key in self.cam_list:
            config = constants.CameraConstants.k_cameras[key]

            if key in self.camera_dict:
                cam_data = self.camera_dict[key]
                topic = config['topic_name']
                cam_rot = config.get('rotation', 0)
                cam_fov = config.get('fov', 90)

                # 1. Calculate Real Visibility & Data
                targets = 0
                dist = 0
                rot = 0
                strafe = 0

                visible_targets = []
                
                # Determine candidates based on camera type
                candidates = []
                if config['type'] == 'hsv':
                    candidates = [{'pos': gp['pos'], 'facing': None, 'id': None} for gp in gamepieces if gp['active']]
                elif config['type'] == 'tags':
                    candidates = [{'pos': p.translation(), 'facing': p.rotation(), 'id': tag_id}
                                  for p, tag_id in zip(self.tag_poses, self.tag_ids)]

                # Unified detection logic
                for candidate in candidates:
                    target_pos = candidate['pos']
                    target_facing = candidate['facing']

                    # Check tag visibility angle if applicable
                    if target_facing is not None:
                        # Vector from tag to robot
                        vec_tag_to_robot = robot_pose.translation() - target_pos
                        angle_to_robot = vec_tag_to_robot.angle()
                        
                        # Angle difference between tag normal and vector to robot
                        angle_diff = (angle_to_robot - target_facing).degrees()
                        angle_diff = (angle_diff + 180) % 360 - 180
                        
                        if abs(angle_diff) > constants.SimConstants.k_tag_visibility_angle:
                            continue

                    vec_to_target = target_pos - robot_pose.translation()
                    # Sitting exactly on the target there is no direction to it: 2027a7's
                    # Translation2d.angle() returns None for a zero vector (2026 logs an error
                    # and returns 0), and a camera cannot see what is under the robot anyway.
                    if vec_to_target.norm() < 1e-6:
                        continue

                    angle_robot_relative = (vec_to_target.angle() - robot_pose.rotation()).degrees()
                    angle_robot_relative = (angle_robot_relative + 180) % 360 - 180

                    angle_cam_relative = angle_robot_relative - cam_rot
                    angle_cam_relative = (angle_cam_relative + 180) % 360 - 180

                    d = vec_to_target.norm()
                    if abs(angle_cam_relative) < (cam_fov / 2.0) and d < constants.SimConstants.k_cam_distance_limit:
                        s = d * math.sin(math.radians(angle_cam_relative))
                        visible_targets.append({'dist': d, 'rot': angle_cam_relative, 'strafe': s, 'id': candidate['id'],
                                                'robot_relative': vec_to_target.rotate_by(-robot_pose.rotation())})

                if visible_targets:
                    closest = min(visible_targets, key=lambda x: x['dist'])
                    targets = len(visible_targets)
                    dist = closest['dist']
                    rot = closest['rot']
                    strafe = closest['strafe']

                cam_data['frames'] += 1
                cam_data['frames_pub'].set(cam_data['frames'])
                cam_data['targets_pub'].set(targets)
                cam_data['distance_pub'].set(dist)
                cam_data['strafe_pub'].set(strafe)
                cam_data['rotation_pub'].set(rot)

                if 'field_positions_pub' in cam_data:
                    # nearest first, placed on the field from the robot's own pose estimate
                    estimate = self.robot_estimate_sub.get()
                    cam_data['field_positions_pub'].set([
                        estimate.translation() + t['robot_relative'].rotate_by(estimate.rotation())
                        for t in sorted(visible_targets, key=lambda t: t['dist'])])

                # A tag camera with a tag in view also reports where the ROBOT is, which is
                # what Swerve fuses.  A real camera works that out from the tag; we know the
                # answer, so it is ground truth, with no noise.  hil_snap.py must never snap
                # to this - it only listens to cameras we do not publish.
                if visible_targets and 'pose_pub' in cam_data:
                    cam_data['pose_pub'].set([closest['id'], robot_pose.x, robot_pose.y, 0.0,
                                              0.0, 0.0, robot_pose.rotation().radians()])

            # 3. Update FOV Visualization
            self._update_fov_visualization(key, robot_pose, config, self.show_fov_subs[key].get())

    def _update_fov_visualization(self, key, robot_pose, config, show_fov):
        if not show_fov:  # toggled per camera on <sim_prefix>/FOV/<camera>_show_fov (commands/sim_show_fov.py)
            self.fov_objects[key].set_poses([])
            return

        robot_pos = robot_pose.translation()
        robot_rot_rad = robot_pose.rotation().radians()
        fov_dist = constants.SimConstants.k_cam_distance_limit

        cam_rot_rad = math.radians(config.get('rotation', 0))
        cam_fov_rad = math.radians(config.get('fov', 90))

        left_edge_angle = robot_rot_rad + cam_rot_rad - (cam_fov_rad / 2)
        right_edge_angle = robot_rot_rad + cam_rot_rad + (cam_fov_rad / 2)

        p1 = robot_pos
        p2 = robot_pos + Translation2d(fov_dist * math.cos(left_edge_angle), fov_dist * math.sin(left_edge_angle))
        p3 = robot_pos + Translation2d(fov_dist * math.cos(right_edge_angle), fov_dist * math.sin(right_edge_angle))

        self.fov_objects[key].set_poses([Pose2d(p1, Rotation2d()), Pose2d(p2, Rotation2d()), Pose2d(p3, Rotation2d())])

    def _run_blink_test(self, now):
        # 60s cycle for camera disconnection
        num_physical = len(self.physical_cameras)
        if num_physical > 0:
            time_per_cam = 60.0 / num_physical
            off_index = int(now / time_per_cam) % num_physical
            topic_off = self.physical_cameras[off_index]
        else:
            topic_off = None

        # Cycle through logical cameras one at a time (2s per camera)
        if len(self.cam_list) > 0:
            key_target_on = self.cam_list[int(now / 2.0) % len(self.cam_list)]
        else:
            key_target_on = None

        for key in self.cam_list:
            if key not in self.camera_dict:
                continue

            cam_data = self.camera_dict[key]
            config = constants.CameraConstants.k_cameras[key]
            topic = config['topic_name']

            is_connected = (topic != topic_off)


            pub_list = [cam_data['targets_pub'], cam_data['distance_pub'],
                        cam_data['strafe_pub'], cam_data['rotation_pub']]

            if is_connected:
                cam_data['frames'] += 1
                cam_data['frames_pub'].set(cam_data['frames'])

                if key == key_target_on:  # set all the target data to the position in the list
                    data = cam_data['offset'] + 1
                    for pub in pub_list:
                        pub.set(data)
                else:
                    for pub in pub_list:  # set all the target data to 0
                        pub.set(0)
            else:
                # Disconnected: freeze frames, clear targets
                for pub in pub_list:  # set all the target data to 0
                    pub.set(0)