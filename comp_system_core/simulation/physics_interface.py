"""
Local replacement for pyfrc's PhysicsInterface, plus the loop that drives physics.py.

WHY THIS FILE EXISTS
--------------------
Through 2026, `robotpy sim` was pyfrc's command (pyfrc.mains.cli_sim:PyFrcSim).  It built a
PhysicsInterface, loaded <project>/physics.py, instantiated our PhysicsEngine and called
update_sim() for us every loop.  None of that was ever WPILib - it is a robotpy-only
framework that predates WPILib having real simulation support.

In 2027 `sim` and `test` moved OUT of pyfrc and INTO wpilib core
(wpilib._impl.cli_sim:RobotSim).  Read that file: it loads halsim_gui, loads sim extensions,
and calls robot_class.main().  There is no physics support of any kind and nothing looks for
physics.py.  So physics.py was simply never imported, our Field2d was never constructed,
nothing published /SmartDashboard/Field, and the sim GUI had no field to offer.

Installing pyfrc does NOT fix this.  pyfrc 2027.0.0a2 declares its entry points under the
group [robotpy], but robotpy-cli 2027 only scans [robotpy_cli.2027] (see robotpy/main.py,
the `for entry_point in entry_points(group="robotpy_cli.2027")` loop).  Its `sim` command is
silently ignored - the CLI never sees it.  That release also still pins robotpy-cli~=2024.0.
It is a placeholder, not a working build.  Do not install it expecting the field back.

WHAT THIS IS
------------
PhysicsInterface reimplements the three methods our code actually calls - get_pose(),
move_robot() and drive() - with the same semantics pyfrc had.  PhysicsEngineHost owns the
engine and the timing, and is pumped from MyRobot.simulationInit()/simulationPeriodic(),
which is where WPILib has expected simulation code to live since 2020.

THE GROUND TRUTH POSE lives in this object's Field2d, exactly as pyfrc did it: get_pose() is
field.getRobotPose(), and move_robot()/drive() mutate it.  physics.py then creates its OWN
Field2d, publishes it on "Field", and copies the ground truth into it each update_sim so it
can hang the Target/ShotLine objects off the same widget.

That is why `publish` defaults to False here.  pyfrc DID publish its field on "Field", so
under pyfrc two different Sendables were pushed to one key and the later one - physics.py's -
won.  Leaving ours unpublished produces the identical picture without relying on that
collision resolving the way we want.  Pass publish=True only if you drop physics.py's field.

THIS IS A BRIDGE, NOT THE DESTINATION.  See the note at the bottom of this file.
"""

import typing

import wpilib
from wpimath import ChassisVelocities, Pose2d, Transform2d, Twist2d


class PhysicsInterface:
    """The subset of pyfrc's PhysicsInterface that this project uses."""

    def __init__(self, publish: bool = False):
        self.field = wpilib.Field2d()
        if publish:
            wpilib.SmartDashboard.putData("Field", self.field)

    def drive(self, speeds: ChassisVelocities, tm_diff: float) -> Pose2d:
        """Integrate chassis velocities over tm_diff and move the ground truth pose."""
        # 2027 removed Pose2d.exp(twist).  The replacement is pose.transformBy(twist.exp()),
        # where Twist2d.exp() now returns the Transform2d.  Same arc integration, verified
        # against the old result - do not "simplify" this to a straight-line translation.
        twist = Twist2d(
            dx=speeds.vx * tm_diff,
            dy=speeds.vy * tm_diff,
            dtheta=speeds.omega * tm_diff,
        )
        pose = self.field.getRobotPose().transformBy(twist.exp())
        self.field.setRobotPose(pose)
        return pose

    def move_robot(self, transform: Transform2d) -> Pose2d:
        """Move the ground truth pose by a relative transform."""
        pose = self.field.getRobotPose() + transform
        self.field.setRobotPose(pose)
        return pose

    def get_pose(self) -> Pose2d:
        """The ground truth pose - what the robot is ACTUALLY doing, not what odometry thinks."""
        return self.field.getRobotPose()


class PhysicsEngineHost:
    """Owns physics.py's PhysicsEngine and does the timing pyfrc used to do for us."""

    def __init__(self, robot: wpilib.RobotBase):
        # Deferred import, and it has to stay deferred: physics.py does `from robot import
        # MyRobot` at module scope, so importing it from robot.py's top level is a circular
        # import.  By the time simulationInit() runs, robot.py is fully loaded and this is safe.
        from physics import PhysicsEngine

        self.interface = PhysicsInterface()
        self.engine = PhysicsEngine(self.interface, robot)
        self.last_tm: typing.Optional[float] = None

    def update(self) -> None:
        now = wpilib.Timer.getTimestamp()
        if self.last_tm is None:
            # Skip the first call: there is no previous timestamp to difference against, and
            # pyfrc skipped it too.  Handing update_sim a tm_diff of 0 divides by zero in
            # some of the sim models.
            self.last_tm = now
            return
        tm_diff = now - self.last_tm
        if tm_diff <= 0:
            return
        self.last_tm = now
        self.engine.update_sim(now, tm_diff)


# ---------------------------------------------------------------------------
# WHAT THIS IS A BRIDGE *TO*
#
# The WPILib-standard way - what Java and C++ teams have always done, because they never
# had physics.py - is that simulation lives INSIDE the subsystem it simulates:
#
#   * commands2.Subsystem.simulationPeriodic() is called for every registered subsystem by
#     CommandScheduler.run() whenever RobotBase.isSimulation().  It is already wired up; we
#     have simply never overridden it.
#   * Each subsystem owns its own plant model from wpilib.simulation - DCMotorSim,
#     FlywheelSim, SingleJointedArmSim, ElevatorSim - and writes the result back into its
#     motor controllers' sim state objects (TalonFXSimState from phoenix6, SparkSim from
#     robotpy-rev).  The robot code above then reads those exactly as it reads real hardware.
#   * Field2d is published by the drive subsystem, from the pose estimator, on the real
#     robot too - not only in sim.  That is why Java teams see a field without any of this.
#
# Migrating means moving swerve_sim.py into Swerve.simulationPeriodic(), gamepiece/vision
# sim into their own subsystems, and deleting this file along with physics.py.  The win is
# that the ground truth pose stops being a thing only the simulator knows, and sim code stops
# reaching across the whole robot through `self.robot.container.<anything>`.
# ---------------------------------------------------------------------------
