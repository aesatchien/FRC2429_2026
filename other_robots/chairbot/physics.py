#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the notes for the other physics sample
#

import math
import wpilib
import wpilib.simulation as simlib
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from wpimath.kinematics import (DifferentialDriveKinematics,
                                DifferentialDriveKinematicsBase,
                                DifferentialDriveOdometry,
                                DifferentialDriveOdometryBase,
                                DifferentialDriveWheelPositions,
                                DifferentialDriveWheelSpeeds)
import wpimath.geometry as geo
import hal
import ntcore
import rev
from wpimath.system.plant import DCMotor
from pyfrc.physics.units import units

import constants

from robot import MyRobot


class PhysicsEngine:
    """
    Simulates our tank drive from the commands sent to the left and right leaders
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot

        self.kinematics = DifferentialDriveKinematics(trackWidth=0.7)

        # set up LEDs - apparently not necessary - glass gui grabs the default one and you can show it
        # self.ledsim = simlib.AddressableLEDSim()

        # TODO: determine if we need these
        self.analogs = [simlib.AnalogInputSim(i) for i in range(4)]
        self.analog_offsets = []

        # the leaders' commands drive the sim, and the spark sims keep the robot's encoders up to date
        self.max_meters_per_second = 4  # wheel speed at a motor command of +/-1
        self.drive = robot.container.drive
        self.spark_l = rev.SparkMaxSim(self.drive.drive_l1, DCMotor.NEO(2))
        self.spark_r = rev.SparkMaxSim(self.drive.drive_r1, DCMotor.NEO(2))

        # sensors

        # set up the initial location of the robot on the field
        self.x, self.y = constants.k_start_x, constants.k_start_y
        self.theta = 0
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d())
        self.physics_controller.move_robot(geo.Transform2d(self.x, self.y, 0))

        # Create a Mechanism2d display of a chairbot
        self.chairbot_side_mech2d = wpilib.Mechanism2d(40, 60, backgroundColor=wpilib.Color8Bit(wpilib.Color.kNavy))  # window
        self.chassis_base = self.chairbot_side_mech2d.getRoot("chassis_base", 5, 13)
        self.chassis_ligament = self.chassis_base.appendLigament(
            "chassis", 28, 0, 12, wpilib.Color8Bit(wpilib.Color.kGray))
        self.chair_base = self.chairbot_side_mech2d.getRoot("chair_base", 12, 13)
        self.chair_ligament = self.chair_base.appendLigament(
            "chair", 18, 90, 12, wpilib.Color8Bit(wpilib.Color.kOrange))
        self.chair_rear_wheel_base = self.chairbot_side_mech2d.getRoot("rear_wheel_base", 5, 10)
        self.side_rear_wheel = self.chair_rear_wheel_base.appendLigament(
            "rear_wheel", 5, 0, 24, wpilib.Color8Bit(wpilib.Color.kWhiteSmoke))
        self.chair_front_wheel_base = self.chairbot_side_mech2d.getRoot("front_wheel_base", 18, 10)
        self.side_front_wheel = self.chair_front_wheel_base.appendLigament(
            "front_wheel", 5, 0, 24, wpilib.Color8Bit(wpilib.Color.kWhiteSmoke))
        # make the sim available to the dash
        wpilib.SmartDashboard.putData("ChairBotSim", self.chairbot_side_mech2d)

        # Create a Mechanism2d display of an Arm
        self.mech2d = wpilib.Mechanism2d(40, 60, backgroundColor=wpilib.Color8Bit(wpilib.Color.kNavy))
        self.armBase = self.mech2d.getRoot("ArmBase", 30, 20)
        self.chassisBase = self.mech2d.getRoot("chassisBase", 5, 13)
        self.chassis = self.chassisBase.appendLigament(
            "chassis", 28, 0, 12, wpilib.Color8Bit(wpilib.Color.kGray))
        self.intake_mech = self.chassis.appendLigament(
            "Intake", 10, 180, 10, wpilib.Color8Bit(wpilib.Color.kDarkRed)
        )
        self.crank_arm_mech = self.armBase.appendLigament(
            "Crank Arm Tower", 22, 90, 12, wpilib.Color8Bit(wpilib.Color.kDodgerBlue)
        )
        self.shooter_arm_mech = self.crank_arm_mech.appendLigament(
            "Shooter Arm", 19, 175, 20, wpilib.Color8Bit(wpilib.Color.kYellow)
        )
        self.indexer_mech = self.shooter_arm_mech.appendLigament(
            "Indexer", 5, 180, 15, wpilib.Color8Bit(wpilib.Color.kDarkRed)
        )
        self.note_mech = self.indexer_mech.appendLigament(
            "Spacer", 8, 0, 10, wpilib.Color8Bit(wpilib.Color.kOrange)
        )
        self.flywheel_mech = self.note_mech.appendLigament(
            "Flywheel", 6, 0, 10, wpilib.Color8Bit(wpilib.Color.kDarkRed)
        )

        # make the sim available to the dash
        wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

        # self.arm_motor: rev.CANSparkMax = robot.container.crank_arm.motor

        # simulating if the robot has a note
        self.has_note = True

    def robot_on_note(self):  # determine if robot is over a note
        note_locations = [(2.94, 7.00), (2.94, 5.57), (2.94, 4.10),
                          (8.33, 7.46), (8.33, 5.76), (8.33, 4.10 ), (8.33, 2.42), (8.33, 0.76),
                          (13.73, 7.00), (13.73, 5.57), (13.73, 4.10)]
        robot_radius = 0.5
        on_note = False
        for loc in note_locations:
            if distance(p1=(self.x, self.y), p2=loc) < robot_radius:
                on_note = True
        return on_note


    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Not sure why this is necessary but it is- LHACK 3/12/24
        simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)

        # convert the motor commands to wheel speeds - the real sparks stop when disabled, so do that here too
        left_velocity, right_velocity = 0, 0
        if wpilib.DriverStation.isEnabled():
            left_velocity = self.max_meters_per_second * max(-1.0, min(1.0, self.drive.drive_l1.get()))
            right_velocity = self.max_meters_per_second * max(-1.0, min(1.0, self.drive.drive_r1.get()))

        diff_wheel_speeds = DifferentialDriveWheelSpeeds(left=left_velocity, right=right_velocity)
        # using our own kinematics to update the chassis speeds
        speeds = self.kinematics.toChassisSpeeds((diff_wheel_speeds))

        # update the sim's robot
        self.physics_controller.drive(speeds, tm_diff)

        # send our poses to the dashboard so we can use it with our trackers
        pose = self.physics_controller.get_pose()
        self.x, self.y, self.theta = pose.X(), pose.Y(), pose.rotation().degrees()

        # update the robot's encoders
        self.spark_l.iterate(left_velocity, 12, tm_diff)
        self.spark_r.iterate(right_velocity, 12, tm_diff)

        wpilib.SmartDashboard.putNumberArray('sim_pose', [self.x, self.y, self.theta])
        wpilib.SmartDashboard.putNumberArray('drive_pose', [self.x, self.y, self.theta])  # need this for 2429 python dashboard to update

        # update the wheels
        wheel_mechs = [self.side_rear_wheel, self.side_front_wheel]
        wheel_color = None
        if left_velocity < -0.05:
            wheel_color = wpilib.Color8Bit(wpilib.Color.kDarkRed)
        elif left_velocity > 0.05:
            wheel_color = wpilib.Color8Bit(wpilib.Color.kGreen)
        else:
            wheel_color = wpilib.Color8Bit(wpilib.Color.kGhostWhite)
        for wheel_mech in wheel_mechs:
            wheel_mech.setColor(wheel_color)

        # Update the arm

        # update the shooter, indexer and intake sims
        #indexer_on = math.fabs(self.spark_dict['indexer']['output'].value) > 0.1
        #flywheel_on = math.fabs(self.spark_dict['t_shooter']['velocity'].value) > 0.1 or math.fabs(self.spark_dict['t_shooter']['output'].value) > 0.1
        #intake_on = math.fabs(self.spark_dict['intake']['output'].value) > 0.1
        #flywheel_color = wpilib.Color8Bit(wpilib.Color.kLimeGreen) if flywheel_on else wpilib.Color8Bit(wpilib.Color.kDarkRed)
        #indexer_color = wpilib.Color8Bit(wpilib.Color.kLimeGreen) if indexer_on else wpilib.Color8Bit(wpilib.Color.kDarkRed)
        #intake_color = wpilib.Color8Bit(wpilib.Color.kLimeGreen) if intake_on else wpilib.Color8Bit(wpilib.Color.kDarkRed)
        #note_color = wpilib.Color8Bit(wpilib.Color.kOrange) if self.has_note else wpilib.Color8Bit(wpilib.Color.kYellow)
        #self.flywheel_mech.setColor(flywheel_color)
        #self.indexer_mech.setColor(indexer_color)
        #self.intake_mech.setColor(intake_color)
        #self.note_mech.setColor(note_color)

        # update the vision
        # ring_dist, ring_rot = self.distance_to_ring()
        #wpilib.SmartDashboard.putNumber('/sim/hub_dist', round(ring_dist, 2))
        #wpilib.SmartDashboard.putNumber('/sim/hub_rot', round(ring_rot, 2))
        #self.targets_entry.setDouble(1)
        #self.distance_entry.setDouble(ring_dist)
        #self.strafe_entry.setDouble(0)
        #self.rotation_entry.setDouble(self.theta - ring_rot)
        #self.timestamp_entry.setDouble(wpilib.Timer.getFPGATimestamp())  # pretend the camera is live


        # update note captured if we drive over a note 20240404 CJH
        # if self.robot.container.shooter.shooter_on:
        #     self.has_note = False  # get rid of a note
        # if not self.has_note:
        #     self.has_note = self.robot_on_note()  # add one if we drive over it
        # self.robot.container.shooter.ring_loaded = self.has_note  # update the shooter to return that we have a ring

def crank_angle_to_sim(crank_angle):
    # for us 90 is straight up and the angle is positive CW (looking at robot from the right)
    # for sim 90 is up but we are positive CCW - so take 90=90 but we're reversed from there
    return 90 + (90 - math.degrees(crank_angle))  # same as 180 - crank_angle
    pass

def shooter_angle_to_sim(shooter_angle):
    # for the shooter our zero (horizontal or perpendicular to the arm) is the sim's 90
    return 90 - math.degrees(shooter_angle)

def distance(p1, p2):
    return math.sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )
