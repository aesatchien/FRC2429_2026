import commands2
import wpilib
from wpilib import GenericHID

from helpers import joysticks as js
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)  # note this sets self.start_time for you
class RumbleCommand(commands2.Command):
    """
    Buzz a controller.  Without rumble_time it starts rumbling and does not stop until you call
    it again; with rumble_time it rumbles for that long and stops itself.

    NOTE 2026: this used to reach for container.driver_command_controller and container.timer,
    neither of which exists on RobotContainer - five references, all of which would have raised
    AttributeError the moment anything scheduled this.  It takes the controller directly now
    (defaulting to the driver, like every other command that needs one) and owns its own Timer,
    so it has no dependency on the container's shape at all.
    """

    def __init__(self, rumble_amount: float, left_rumble: bool, right_rumble: bool,
                 rumble_time=None, controller=None, indent=0) -> None:
        super().__init__()
        self.setName('Rumble command')
        self.indent = indent

        self.controller = controller if controller is not None else js.driver_controller
        self.rumble_amount = rumble_amount
        self.rumble_time = rumble_time
        self.left_rumble = left_rumble
        self.right_rumble = right_rumble
        self.timer = wpilib.Timer()

        if (not left_rumble) and (not right_rumble):
            raise ValueError("why are you making a rumblecommand with no rumble")

    def runsWhenDisabled(self):
        return True

    def _set_rumble(self, amount: float) -> None:
        hid = self.controller.getHID()
        if self.left_rumble:
            hid.setRumble(GenericHID.RumbleType.kLeftRumble, amount)
        if self.right_rumble:
            hid.setRumble(GenericHID.RumbleType.kRightRumble, amount)

    def initialize(self) -> None:
        self.timer.restart()
        self._set_rumble(self.rumble_amount)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.rumble_time:
            return self.timer.hasElapsed(self.rumble_time)
        return True

    def end(self, interrupted: bool) -> None:
        # Always stop, including on interrupt - the old version only stopped when rumble_time was
        # set, so the no-timeout mode could leave a controller buzzing indefinitely.
        self._set_rumble(0.0)
