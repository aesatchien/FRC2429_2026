import math
from enum import Enum
import commands2
import wpilib
from wpilib import Timer, DriverStation, PowerDistribution
from wpimath.filter import LinearFilter, MedianFilter
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

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self._init_networktables()

        self._callbacks = []  # Store functions to notify

        # initialize modes and indicators
        self.state = self.State.NONE  # This now calls the setter

        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.states_dict = {state.value["name"]: state for state in self.State}

        # keep track of FMS
        self._last_fms = False

        # ---------- power monitoring filters ----------
        # IIR smooths voltage (slow-changing signal, ~0.5s time constant, 0.04s period = 25Hz read rate)
        self.voltage_filter = LinearFilter.singlePoleIIR(timeConstant=0.1, period=0.04)
        # MedianFilter rejects CAN glitch readings on current without adding lag
        self.current_filter = MedianFilter(3)

        # ---------- power tracking state ----------
        self.cumulative_energy = 0.0
        self._prev_power = 0.0
        self.min_voltage = float('inf')   # reset on enable
        self.max_current = 0.0            # reset on enable
        self._voltage = 0.0
        self._current = 0.0
        self._power = 0.0
        self._brownout_detected = False

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.state_pub = self.inst.getStringTopic(f"{constants.status_prefix}/_robot_state").publish()
        self.fms_pub = self.inst.getBooleanTopic(f"{constants.status_prefix}/_fms_attached").publish()
        self.fms_pub.set(False)
        self.pdh_volt_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_voltage").publish()
        self.pdh_current_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_current").publish()
        self.pdh_power_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_inst_power").publish()
        self.pdh_cumulative_energy_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_tot_energy").publish()
        self.pdh_min_voltage_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_min_voltage").publish()
        self.pdh_max_current_pub = self.inst.getDoubleTopic(f"{constants.status_prefix}/_pdh_max_current").publish()
        self.rio_browned_out_pub = self.inst.getBooleanTopic(f"{constants.status_prefix}/_rio_browned_out").publish()

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

        # fast path: read + filter + integrate at 25Hz (every 2 cycles)
        if self.counter % 2 == 0:
            self._brownout_detected |= wpilib.RobotController.isBrownedOut()
            voltage = self.voltage_filter.calculate(self.pdh.getVoltage())
            current = self.current_filter.calculate(self.pdh.getTotalCurrent())
            power = voltage * current
            # Trapezoidal is second-order accurate (error scales with dt²), left Riemann is first-order (error scales with dt)
            self.cumulative_energy += (self._prev_power + power) / 2 * 0.04  # trapezoidal, 0.04s period
            self._prev_power = power
            self.min_voltage = min(self.min_voltage, voltage)
            self.max_current = max(self.max_current, current)
            # cache for the publish path below
            self._voltage = voltage
            self._current = current
            self._power = power

        # slow path: publish to NT at 5Hz (every 10 cycles)
        if self.counter % 10 == 0:
            self.pdh_volt_pub.set(self._voltage)
            self.pdh_current_pub.set(self._current)
            self.pdh_power_pub.set(self._power)
            self.pdh_cumulative_energy_pub.set(self.cumulative_energy)
            self.pdh_min_voltage_pub.set(self.min_voltage)
            self.pdh_max_current_pub.set(self.max_current)
            self.rio_browned_out_pub.set(self._brownout_detected)
            self._brownout_detected = False  # reset for next window

        if self.counter % 100 == 0:  # let's check if we have connected to the FMS
            fms = DriverStation.isFMSAttached()
            just_connected = fms and not self._last_fms
            if just_connected:
                print("**** Just connected to FMS! ****")
                self.fms_pub.set(True)

            if not fms and self._last_fms:
                print("**** Just lost connection to FMS! ****")
                self.fms_pub.set(False)

            self._last_fms = fms