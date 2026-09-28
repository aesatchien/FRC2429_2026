"""
Where the game pieces are on the simulated field, and which ones the robot has driven over.

This is FIELD state, not a mechanism, so it lives on RobotState (see
RobotState.simulation_periodic) rather than on the intake.  It draws on the shared Field2d
through helpers.dashboard so the camera sim can read the same "Gamepieces" object back and
see what is actually on the floor.
"""

import wpilib
from wpimath import Translation2d, Pose2d, Rotation2d

from helpers import dashboard
from simulation import sim_utils


class GamepieceSim:
    k_field_object = "Gamepieces"

    def __init__(self):
        # Initial locations
        # self.gamepiece_locations = [(2.89, 7.0), (2.89, 5.57), (2.89, 4.1), (8.28, 7.46), (8.28, 5.76), (8.28, 4.1),
        #                             (8.28, 2.42), (8.28, 0.76), (13.68, 7.0), (13.68, 5.57), (13.68, 4.1)]
        self.gamepiece_locations = [(x/10, y/10) for x in range(75, 95, 5) for y in range(20, 63, 5) ]  # rebuild
        self.gamepieces = [{'pos': Translation2d(gl), 'active': True} for gl in self.gamepiece_locations]

        # Registered through dashboard so the a7 hand publisher knows to send it - a bare
        # field.get_object() would draw nothing (see helpers/dashboard.py, _NATIVE_GAP).
        self.gamepiece_obj = dashboard.field_object(self.k_field_object)
        self.on_gamepiece = False
        self.update_field()

    def update(self, robot_pose: Pose2d):
        # Check consumption
        changed = False
        self.on_gamepiece = False
        for gp in self.gamepieces:
            if gp['active']:
                if sim_utils.is_on_gamepiece(robot_pose, gp['pos']):
                    self.on_gamepiece = True
                    gp['active'] = False
                    print(f"Simulation consumed gamepiece at {gp['pos']}")
                    changed = True

        # Auto-reset if all consumed
        if len([gp for gp in self.gamepieces if gp['active']]) == 0:
            self.reset_gamepieces()
            changed = True

        if changed:
            self.update_field()

    def reset_gamepieces(self):
        for gp in self.gamepieces:
            gp['active'] = True
        print(f"Simulation reset all game pieces")
        self.update_field()

    def update_field(self):
        active_poses = [Pose2d(gp['pos'], Rotation2d()) for gp in self.gamepieces if gp['active']]
        self.gamepiece_obj.set_poses(active_poses)

    def get_active_gamepieces(self):
        return self.gamepieces

    @classmethod
    def active_positions_from_field(cls) -> list[Translation2d]:
        """What is on the floor right now, read back off the field - so a consumer (the camera
        sim) needs no reference to this object, only to the shared Field2d."""
        return [pose.translation() for pose in dashboard.field_object(cls.k_field_object).get_poses()]
