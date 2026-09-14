import math

import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
from wpimath import Pose2d, Transform2d
from wpimath.units import inches_to_meters
# pyfrc is gone in 2027 and its `robotpy sim` no longer exists - see
# simulation/physics_interface.py for why this import had to move.
from simulation.physics_interface import PhysicsInterface
import ntcore

from constants import mech_prefix
from robot import MyRobot
import constants
from simulation import sim_utils
from simulation.swerve_sim import SwerveSim
from simulation.gamepiece_sim import GamepieceSim
from simulation.vision_sim import VisionSim
from subsystems.climber import Climber
from helpers import dashboard

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot
        self.container = self.robot.container

        self._init_networktables()
        
        # Create a Field2d for visualization
        self.field = wpilib.Field2d()
        dashboard.SmartDashboard.put_data("Field", self.field)  # should just keep the default one but adds our piece poses
        self.target_object = self.field.get_object("Target")
        self.shotline_object = self.field.get_object("ShotLine")

        # Initialize Simulations
        self.swerve_sim = SwerveSim(physics_controller, robot)
        self.gamepiece_sim = GamepieceSim(self.field)
        self.vision_sim = VisionSim(self.field)

        # Ghost Robot linger state
        self.last_ghost_update_time = 0
        self.ghost_linger_duration = 2.0 # seconds

        # Initial Pose
        self.physics_controller.move_robot(Transform2d(constants.k_start_x, constants.k_start_y, 0))

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.get_default()
        sim_prefix = constants.sim_prefix
        auto_sim_prefix = constants.auto_prefix

        # ground truth Publisher for Simulating Sensors
        self.ground_truth_pub = self.inst.get_struct_topic(f"{sim_prefix}/ground_truth", Pose2d).publish()

        # Ghost Robot Subscribers - used for tracking goals in auto
        self.auto_active_sub = self.inst.get_boolean_topic(f"{auto_sim_prefix}/robot_in_auto").subscribe(False)
        self.goal_pose_sub = self.inst.get_struct_topic(f"{auto_sim_prefix}/goal_pose", Pose2d).subscribe(Pose2d())
        self.shot_line_sub = self.inst.get_struct_array_topic(f"{auto_sim_prefix}/shot_line", Pose2d).subscribe([])


    def update_sim(self, now, tm_diff):

        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        amps = []

        # Update Physics Models
        self.swerve_sim.update(tm_diff)

        simlib.RoboRioSim.set_vin_voltage(simlib.BatterySim.calculate(amps))
        
        # Update Game State
        robot_pose = self.physics_controller.get_pose()
        self.gamepiece_sim.update(robot_pose)
        
        # Update Vision
        active_gamepieces = self.gamepiece_sim.get_active_gamepieces()
        self.vision_sim.update(robot_pose, active_gamepieces)
        
        # Update Field2d
        self.field.set_robot_pose(robot_pose)
        self.ground_truth_pub.set(robot_pose)

        # Update Ghost Robot
        if self.auto_active_sub.get():
            self.target_object.set_pose(self.goal_pose_sub.get())
            self.shotline_object.set_poses(self.shot_line_sub.get())
            self.last_ghost_update_time = now
        else:
            # make it disappear after the ghost timeout
            if now - self.last_ghost_update_time > self.ghost_linger_duration:
                self.target_object.set_poses([])
                self.shotline_object.set_poses([])