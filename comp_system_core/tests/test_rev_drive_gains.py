"""
A REV drive motor asked for a speed must settle near that speed, not at full output.

WHY THIS FILE EXISTS
--------------------
rev 2027a7 deleted the encoder conversion factors, so a Spark's velocity loop now works in
motor RPM.  The drive gains in swerve_constants were left in m/s units after the port
(kP 0.02 per m/s, kV 12 V per free-speed m/s).  Against an RPM setpoint that is about 1270x
too much of both, and the drive saturated for any nonzero request - asked 0.5 m/s, got 5.2.

Nothing noticed, because the comp bot's drive is a Kraken ('ctre').  The REV drive config is
only built into a robot by k_swerve_config = 'comp_vortex' (the swap-a-Vortex-in rollback) or
'practice' - i.e. it would have been found at an event, on the day a Kraken died.

This drives a simulated SparkFlex through the same RevDriveMotor adapter and the same
k_driving_config those robots use.  Simulation only; it has not been run on a real REV drive.
"""

import rev
import wpilib.simulation

from subsystems.motors import RevDriveMotor
from subsystems.swerve_constants import ModuleConstants as mc

k_spare_can_id = 60   # nothing on the robot uses it (REV allows one object per id per process)


def test_rev_drive_motor_tracks_the_requested_speed(container):
    motor = RevDriveMotor(k_spare_can_id, rev.SparkFlex, mc.k_driving_config, label='gain test',
                          position_factor=mc.k_drive_position_factor,
                          velocity_factor=mc.k_drive_velocity_factor)
    ds = wpilib.simulation.DriverStationSim
    was_enabled = ds.get_enabled()
    ds.set_enabled(True)
    ds.notify_new_data()
    try:
        for target in (0.5, 2.0):
            motor.set_velocity_mps(target)
            for _ in range(150):                    # 3 s, far longer than the spin-up
                motor.sim_update(0.02, 12.0)
            got = motor.get_velocity_mps()
            # 20 %: kV is 12 V / the ACTIVE config's free speed (a Kraken's 6000 rpm on the comp
            # bot) while the simulated motor here is a Vortex (6784), so it runs ~13 % fast.
            # The broken gains gave 5.2 m/s for both targets.
            assert abs(got - target) < 0.2 * target, f"asked {target} m/s, got {got:.2f} m/s"
    finally:
        motor.set_velocity_mps(0.0)
        ds.set_enabled(was_enabled)
        ds.notify_new_data()
