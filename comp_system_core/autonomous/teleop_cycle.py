"""
teleop_cycle.py  –  Team 2429  –  2026 Reefscape

Full-teleop automation: repeatedly cycle collect → shoot while hub is active,
or collect-only while hub is inactive.

2026 TELEOP timeline (counts DOWN from 2:20 to 0:00):
    Transition Shift  2:20 – 2:10   10s   both hubs ACTIVE
    Shift 1           2:10 – 1:45   25s   alternating (see HubStatusHelper)
    Shift 2           1:45 – 1:20   25s   alternating
    Shift 3           1:20 – 0:55   25s   alternating
    Shift 4           0:55 – 0:30   25s   alternating
    End Game          0:30 – 0:00   30s   both hubs ACTIVE

FMS Game Data (received at TELEOP start):
    "Red"  → Red scored more in AUTO (or was randomly chosen by FMS)
              → Red hub INACTIVE in Shift 1, Blue hub ACTIVE
    "Blue" → Blue scored more (or chosen) → Blue hub INACTIVE in Shift 1, Red ACTIVE
    ""     → not yet received; treat as hub ACTIVE (conservative default)

Usage – add to robotcontainer.py:

    from autonomous.teleop_cycle import TeleopCycle

    # In bind_driver_buttons (or wherever you want to trigger it):
    js.some_button.whileTrue(TeleopCycle(container=self))

    # OR schedule it as the swerve default command for full-match automation
    # (replace DriveByJoystickSubsystemTargeting with this when you want hands-off)
"""

import wpilib
import commands2
from wpimath.geometry import Pose2d

import constants
from commands.auto_to_pose_clean import AutoToPoseClean
from commands.intake_deploy import Intake_Deploy
from commands.intake_set_rpm import Intake_Set_RPM
from commands.shooting_command import ShootingCommand
from helpers import joysticks as js
from helpers.log_command import log_command


# ---------------------------------------------------------------------------
# Hub-status helper
# ---------------------------------------------------------------------------

class HubStatusHelper:
    """
    Determines whether OUR alliance hub is currently active.

    Call is_hub_active() at any time during TELEOP; it reads DriverStation
    match time and FMS game data to decide.

    FMS Game Data format (2026):
        "R"  – Red alliance hub is INACTIVE first (Shift 1)
        "B"  – Blue alliance hub is INACTIVE first (Shift 1)
        ""   – not yet available

    Shift alternation pattern (starting from who is INACTIVE in Shift 1):
        Shift 1: loser=INACTIVE, winner=ACTIVE
        Shift 2: loser=ACTIVE,   winner=INACTIVE
        Shift 3: loser=INACTIVE, winner=ACTIVE
        Shift 4: loser=ACTIVE,   winner=INACTIVE
    """

    # TELEOP match time boundaries (countdown, seconds remaining)
    TRANSITION_END = 130.0    # 2:10 remaining – shifts begin
    SHIFT_1_END    = 105.0    # 1:45
    SHIFT_2_END    =  80.0    # 1:20
    SHIFT_3_END    =  55.0    # 0:55
    SHIFT_4_END    =  30.0    # 0:30  – End Game begins

    def __init__(self):
        self._fms_data_cached: str = ""
        self._data_received: bool = False

    def _get_fms_data(self) -> str:
        """Cache FMS game data once it arrives (it's set at TELEOP start)."""
        if not self._data_received:
            data = wpilib.DriverStation.getGameSpecificMessage()
            if data:
                self._fms_data_cached = data
                self._data_received = True
                print(f"[HubStatus] FMS data received: '{data}'")
        return self._fms_data_cached

    def _get_our_alliance(self) -> wpilib.DriverStation.Alliance:
        return wpilib.DriverStation.getAlliance()

    def _current_shift(self, match_time: float) -> int:
        """
        Returns which shift we are in:
            0  = Transition Shift (both active)
            1–4 = Alliance Shift 1–4 (alternating)
            5  = End Game (both active)
           -1  = unknown / not in TELEOP
        """
        if match_time > self.TRANSITION_END:
            return 0   # Transition
        elif match_time > self.SHIFT_1_END:
            return 1
        elif match_time > self.SHIFT_2_END:
            return 2
        elif match_time > self.SHIFT_3_END:
            return 3
        elif match_time > self.SHIFT_4_END:
            return 4
        else:
            return 5   # End Game

    def is_hub_active(self) -> bool:
        """
        Returns True if our hub is currently active (scoring counts).
        Defaults to True when data is unavailable (safe: attempt to score).
        """
        if not wpilib.DriverStation.isTeleopEnabled():
            return True  # in auto or disabled – not our concern here

        match_time = wpilib.DriverStation.getMatchTime()  # seconds remaining
        shift = self._current_shift(match_time)

        # Both hubs active during Transition and End Game
        if shift in (0, 5):
            return True

        fms_data = self._get_fms_data()
        if not fms_data:
            # Data not yet received – be optimistic and try to shoot
            return True

        our_alliance = self._get_our_alliance()
        if our_alliance is None:
            return True  # can't determine – default active

        our_color = (
            "R"
            if our_alliance == wpilib.DriverStation.Alliance.kRed
            else "B"
        )

        # fms_data names the alliance that was INACTIVE in Shift 1
        # (the one that scored MORE in AUTO, per the manual)
        inactive_first = fms_data  # "R" or "B"

        # In odd shifts (1, 3): inactive_first alliance is INACTIVE
        # In even shifts (2, 4): inactive_first alliance is ACTIVE (flipped)
        if shift % 2 == 1:
            our_hub_inactive = (our_color == inactive_first)
        else:
            our_hub_inactive = (our_color != inactive_first)

        active = not our_hub_inactive
        return active

    def seconds_until_next_shift(self) -> float:
        """Rough time until the next hub-status change. Useful for deciding
        whether to start a long pickup cycle or just wait."""
        match_time = wpilib.DriverStation.getMatchTime()
        shift = self._current_shift(match_time)
        boundaries = [self.TRANSITION_END, self.SHIFT_1_END,
                      self.SHIFT_2_END, self.SHIFT_3_END,
                      self.SHIFT_4_END]
        for boundary in boundaries:
            if match_time > boundary:
                return match_time - boundary
        return 0.0

    def reset(self):
        """Call at start of teleop to clear cached FMS data."""
        self._fms_data_cached = ""
        self._data_received = False


# ---------------------------------------------------------------------------
# Single Cycle command
# ---------------------------------------------------------------------------

class _OneCycle(commands2.Command):
    """
    One collect+shoot cycle.  Runs as a plain Command (not a group) so we can
    gate on hub status mid-execution and bail out of the shoot phase early.

    Phases:
        COLLECT  – intake deployed, drive to fuel pickup
        NAVIGATE – drive to shooting position
        SHOOT    – run shooter with targeting until shot or hub goes inactive
        RESET    – stow intake, stop shooter
    """

    _PHASE_COLLECT   = "collect"
    _PHASE_NAVIGATE  = "navigate"
    _PHASE_SHOOT     = "shoot"
    _PHASE_RESET     = "reset"

    def __init__(self, container, hub_helper: HubStatusHelper, indent: int = 0):
        super().__init__()
        self.setName("OneCycle")
        self.container = container
        self.hub = hub_helper
        self.indent = indent

        # We will schedule sub-commands imperatively so we can gate on hub status.
        self._active_cmd: commands2.Command | None = None
        self._phase = self._PHASE_COLLECT
        self._done = False

        # Require all subsystems touched by sub-commands so the scheduler
        # doesn't conflict.
        self.addRequirements(
            container.swerve,
            container.shooter,
            container.intake,
        )

    # ------------------------------------------------------------------
    def _start_phase(self, phase: str) -> None:
        if self._active_cmd is not None:
            self._active_cmd.cancel()
            self._active_cmd = None

        self._phase = phase
        prefix = "    " * self.indent

        if phase == self._PHASE_COLLECT:
            print(f"{prefix}[Cycle] → COLLECT")
            # Deploy intake and drive toward nearest fuel
            self.container.intake.intake_on = True
            cmd = commands2.SequentialCommandGroup(
                Intake_Deploy(intake=self.container.intake, position='down'),
                Intake_Set_RPM(
                    intake=self.container.intake,
                    rpm=constants.IntakeConstants.k_intake_default_rpm,
                    led=self.container.led,
                ),
                AutoToPoseClean(
                    container=self.container,
                    swerve=self.container.swerve,
                    target_pose=None,
                    mode="ball_pickup",
                    from_robot_state=True,
                    control_type='not_pathplanner',
                ).withTimeout(4.0),
            )
            cmd.initialize()
            self._active_cmd = cmd

        elif phase == self._PHASE_NAVIGATE:
            print(f"{prefix}[Cycle] → NAVIGATE to shoot position")
            cmd = commands2.SequentialCommandGroup(
                Intake_Set_RPM(
                    intake=self.container.intake, rpm=0,
                    led=self.container.led,
                ),
                Intake_Deploy(intake=self.container.intake, position='shoot'),
                AutoToPoseClean(
                    container=self.container,
                    swerve=self.container.swerve,
                    target_pose=None,
                    mode="shooting",
                    from_robot_state=True,
                    control_type='not_pathplanner',
                ).withTimeout(4.0),
            )
            cmd.initialize()
            self._active_cmd = cmd

        elif phase == self._PHASE_SHOOT:
            print(f"{prefix}[Cycle] → SHOOT")
            self.container.targeting.start_tracking()
            cmd = ShootingCommand(
                shooter=self.container.shooter,
                targeting=self.container.targeting,
                indent=self.indent + 1,
                auto_timeout=3.5,
            )
            cmd.initialize()
            self._active_cmd = cmd

        elif phase == self._PHASE_RESET:
            print(f"{prefix}[Cycle] → RESET")
            self.container.targeting.stop_tracking()
            cmd = commands2.SequentialCommandGroup(
                Intake_Deploy(intake=self.container.intake, position='down'),
                Intake_Set_RPM(
                    intake=self.container.intake,
                    rpm=constants.IntakeConstants.k_intake_default_rpm,
                    led=self.container.led,
                ),
            )
            cmd.initialize()
            self._active_cmd = cmd

    # ------------------------------------------------------------------
    def initialize(self) -> None:
        self._done = False
        self._start_phase(self._PHASE_COLLECT)

    def execute(self) -> None:
        if self._active_cmd is None:
            self._done = True
            return

        self._active_cmd.execute()

        if self._active_cmd.isFinished():
            self._active_cmd.end(False)
            self._active_cmd = None

            if self._phase == self._PHASE_COLLECT:
                # After collecting, decide: is our hub active?
                if self.hub.is_hub_active():
                    self._start_phase(self._PHASE_NAVIGATE)
                else:
                    # Hub inactive – skip shooting, reset and wait
                    print(f"[Cycle] Hub INACTIVE – skipping shoot phase")
                    self._start_phase(self._PHASE_RESET)

            elif self._phase == self._PHASE_NAVIGATE:
                self._start_phase(self._PHASE_SHOOT)

            elif self._phase == self._PHASE_SHOOT:
                self._start_phase(self._PHASE_RESET)

            elif self._phase == self._PHASE_RESET:
                self._done = True

        elif self._phase == self._PHASE_SHOOT and not self.hub.is_hub_active():
            # Hub went inactive mid-shot – abort shooting immediately
            print(f"[Cycle] Hub became INACTIVE mid-shot – aborting")
            self._active_cmd.cancel()
            self._active_cmd = None
            self._start_phase(self._PHASE_RESET)

    def isFinished(self) -> bool:
        return self._done

    def end(self, interrupted: bool) -> None:
        if self._active_cmd is not None:
            self._active_cmd.cancel()
            self._active_cmd = None
        self.container.targeting.stop_tracking()
        self.container.shooter.stop_shooter()
        # Leave intake deployed/running so next cycle can start collecting


# ---------------------------------------------------------------------------
# HubInactiveWait command – collect/patrol during inactive shift
# ---------------------------------------------------------------------------

class _WaitForActiveHub(commands2.Command):
    """
    Runs while our hub is inactive: keep collecting fuel so we're loaded
    and ready the moment our hub goes active again.
    Ends as soon as hub becomes active (or match time runs low).
    """

    # Stop collecting and go to shoot position with this much time left in
    # the current shift, so we arrive ready when the hub flips active.
    PREP_AHEAD_SECONDS = 3.0

    def __init__(self, container, hub_helper: HubStatusHelper):
        super().__init__()
        self.setName("WaitForActiveHub")
        self.container = container
        self.hub = hub_helper
        self._active_cmd: commands2.Command | None = None
        self._prepping = False
        self.addRequirements(container.swerve, container.intake)

    def initialize(self) -> None:
        self._prepping = False
        print("[WaitActive] Hub inactive – collecting while waiting")
        # Run intake setup imperatively (no .schedule() — we own the requirements)
        deploy = Intake_Deploy(intake=self.container.intake, position='down')
        deploy.initialize()
        deploy.execute()
        deploy.end(False)
        self.container.intake.set_intake_rpm(constants.IntakeConstants.k_intake_default_rpm)
        # Start driving toward fuel
        self._active_cmd = AutoToPoseClean(
            container=self.container,
            swerve=self.container.swerve,
            target_pose=None,
            mode="ball_pickup",
            from_robot_state=True,
            control_type='not_pathplanner',
        ).withTimeout(4.0)
        self._active_cmd.initialize()

    def execute(self) -> None:
        # Run active nav command
        if self._active_cmd is not None:
            self._active_cmd.execute()
            if self._active_cmd.isFinished():
                self._active_cmd.end(False)
                self._active_cmd = None

        # Pre-position to shoot when hub is about to flip active
        time_left_in_shift = self.hub.seconds_until_next_shift()
        if time_left_in_shift <= self.PREP_AHEAD_SECONDS and not self._prepping:
            self._prepping = True
            print("[WaitActive] Pre-positioning for upcoming active shift")
            if self._active_cmd is not None:
                self._active_cmd.end(True)
            self._active_cmd = AutoToPoseClean(
                container=self.container,
                swerve=self.container.swerve,
                target_pose=None,
                mode="shooting",
                from_robot_state=True,
                control_type='not_pathplanner',
            ).withTimeout(3.0)
            self._active_cmd.initialize()

    def isFinished(self) -> bool:
        return self.hub.is_hub_active()

    def end(self, interrupted: bool) -> None:
        if self._active_cmd is not None:
            self._active_cmd.end(True)
            self._active_cmd = None


# ---------------------------------------------------------------------------
# Top-level TeleopCycle command
# ---------------------------------------------------------------------------

class TeleopCycle(commands2.Command):
    """
    Full teleop automation.  Runs a shoot-collect cycle continuously,
    pausing to collect/position during inactive shifts.

    Schedule this with whileTrue() on a button, or use it as the swerve
    default command for fully hands-off operation.

        js.driver_start.whileTrue(TeleopCycle(container=self))

    The command runs indefinitely (isFinished always returns False) and
    cleans up when interrupted (button released or match ends).
    """

    def __init__(self, container, indent: int = 0):
        super().__init__()
        self.setName("TeleopCycle")
        self.container = container
        self.indent = indent
        self.hub = HubStatusHelper()
        self._current: commands2.Command | None = None

        self.addRequirements(
            container.swerve,
            container.shooter,
            container.intake,
        )

    def initialize(self) -> None:
        self.hub.reset()
        print("[TeleopCycle] Started")
        self._launch_next()

    def _launch_next(self) -> None:
        """Pick and initialize the next sub-command based on hub status."""
        if self._current is not None:
            self._current.cancel()

        if self.hub.is_hub_active():
            self._current = _OneCycle(
                container=self.container,
                hub_helper=self.hub,
                indent=self.indent + 1,
            )
        else:
            self._current = _WaitForActiveHub(
                container=self.container,
                hub_helper=self.hub,
            )
        self._current.initialize()

    def execute(self) -> None:
        if self._current is None:
            self._launch_next()
            return

        self._current.execute()

        if self._current.isFinished():
            self._current.end(False)
            self._current = None
            self._launch_next()

    def isFinished(self) -> bool:
        # Runs until interrupted (button released / teleop ends)
        return False

    def end(self, interrupted: bool) -> None:
        print(f"[TeleopCycle] Ended (interrupted={interrupted})")
        if self._current is not None:
            self._current.end(True)
            self._current = None
        # Safe state — call subsystem methods directly, never .schedule() here
        self.container.targeting.stop_tracking()
        self.container.shooter.stop_shooter()
        self.container.intake.stop_intake()
