import math
from enum import Enum
import commands2
from wpilib import Timer
from wpimath.geometry import Rotation2d
import ntcore
import constants
from constants import LedConstants


# TODO - do something better than putting a callback in LED but no polling


class RobotState(commands2.Subsystem):
    """ Robot state
        One place to store our current goals (maybe should go into LED?)

        Need to know target (e.g. processor, L3, etc)
        From this you can calculate: Target height, Target angle
    """

    class State(Enum):
        """ State class is for showing current robot goal/mode """
        # can I generate this programmatically from the constants file's list of positions? Some of it.
        TRACKING = {'name': 'tracking', 'lit_leds': LedConstants.k_led_count, 'mode': 'none'}
        SHOOTING = {'name': 'shooting', 'lit_leds': LedConstants.k_led_count, 'mode': 'none'}
        INTAKING = {'name': 'intaking', 'lit_leds': -1 + LedConstants.k_led_count // 8, 'mode': 'none'}
        DEFENDING = {'name': 'defending', 'lit_leds': -2 + 1 * LedConstants.k_led_count // 4, 'mode': 'none'}
        CLIMBING = {'name': 'climbing', 'lit_leds': -3 + 3 * LedConstants.k_led_count // 8, 'mode': 'none'}
        NONE = {'name': 'NONE', 'lit_leds': LedConstants.k_led_count, 'mode': 'none'}

    def __init__(self):
        super().__init__()
        self.setName('RobotState')
        # try to start all the subsystems on a different count so they don't all do the periodic updates at the same time
        self.counter = constants.RobotStateConstants.k_counter_offset

        self._init_networktables()

        self._callbacks = []  # Store functions to notify

        # initialize modes and indicators
        self.state = self.State.NONE  # This now calls the setter

        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.states_dict = {state.value["name"]: state for state in self.State}

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.state_pub = self.inst.getStringTopic(f"{constants.status_prefix}/_robot_state").publish()

    # put in a callback so the logic to LED is not circular
    def register_callback(self, callback):
        """ Allow external systems (like LED) to register for updates. """
        self._callbacks.append(callback)

    def _notify_callbacks(self):
        """ Notify all registered callbacks when RobotState updates. """
        for callback in self._callbacks:
            callback(self.state)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state: State):
        # getattr() is a safe way to get an attribute. If `self._state` doesn't exist
        # on the first run, it will use `self.State.NONE` as a default value
        # instead of crashing.
        self.prev_state = getattr(self, '_state', self.State.NONE)
        self._state = new_state
        self._notify_callbacks()  # Call all registered callbacks
        print(f'State set to {new_state.value["name"]} at {Timer.getFPGATimestamp():.1f}s')
        self.state_pub.set(self._state.value['name'])


    def periodic(self):
        self.counter += 1  # Increment the main counter
        if self.counter % 10 == 0:  # Execute every 5 cycles (10Hz update rate)
            pass