"""
Once a second, read the state of every CAN bus and publish it, so it lands in the .wpilog and Scope2429 can plot it.

WHY THIS EXISTS
---------------
Phoenix printed the CAN utilization of the Kraken bus ONCE, at start-up, when nothing was driving (0.0 %), and nothing
else about the buses was logged.  So after a match with lag, dropped controllers or brownout-like resets there was no
record of whether a bus was saturated, had gone bus-off, or was overflowing its transmit queue.

WHAT
----
RobotController.get_can_status(port) gives, for each bus: utilization, bus-off count, TX-full count, receive-error count and
transmit-error count.  The three SystemCore buses (constants.k_can_bus_*): can_s0 = swerve drive Krakens, can_s1 = swerve
turn SparkFlexes, can_s2 = shooter, intake, PDH.  Published as plain NetworkTables doubles under constants.can_prefix:

    <can_prefix>/can_s0/utilization       as WPILib reports it (a percentage; see below)
    <can_prefix>/can_s0/bus_off_count      <- these four are running totals since boot; a rise is the event
    <can_prefix>/can_s0/tx_full_count
    <can_prefix>/can_s0/rx_errors
    <can_prefix>/can_s0/tx_errors
    <can_prefix>/can_s0/phoenix_utilization_pct   (drive bus only) Phoenix's own reading, x100

and the same for can_s1 and can_s2.  NetworkTables values go into the .wpilog (DataLogManager logs NT), so no extra log file.

UNIT CHECK NEEDED ON THE ROBOT.  WPILib's header says "bus utilization percentage" but not whether that is 0-100 or 0-1.
phoenix_utilization_pct is there to settle it: on can_s0 the two should agree.  Scope2429 assumes 0-100 unless every value in
the log is <= 1.0, in which case it multiplies by 100.

A rise in bus-off or TX-full is printed (so it is in the console log too) as it happens.  Utilization is not printed: without
the unit confirmed a threshold would be a guess.

Never raises: a diagnostic must not be able to stop the robot.
"""

import ntcore
import wpilib

import constants


class CanMonitor:
    def __init__(self, period_loops: int = 50, prefix: str = constants.can_prefix, status_fn=None, phoenix_fn=None) -> None:
        self.period = period_loops
        self.counter = 0
        self.buses = {'can_s0': constants.k_can_bus_drive, 'can_s1': constants.k_can_bus_turn, 'can_s2': constants.k_can_bus_other}
        self._status = status_fn or wpilib.RobotController.get_can_status      # injectable for tests
        self._phoenix = phoenix_fn if phoenix_fn is not None else self._phoenix_pct
        self._phoenix_bus = None
        self._last = {}                                                         # bus -> (bus_off, tx_full) at the last sample
        inst = ntcore.NetworkTableInstance.get_default()
        self.pubs = {name: {k: inst.get_double_topic(f'{prefix}/{name}/{k}').publish()
                            for k in ('utilization', 'bus_off_count', 'tx_full_count', 'rx_errors', 'tx_errors')}
                     for name in self.buses}
        self.phoenix_pub = inst.get_double_topic(f'{prefix}/can_s0/phoenix_utilization_pct').publish()

    def periodic(self) -> None:
        """Call every robot loop (20 ms); it samples once per `period_loops`."""
        self.counter += 1
        if self.counter % self.period:
            return
        try:
            self.sample()
        except Exception as e:  # noqa - never the thing that stops the robot
            if self.counter // self.period <= 3:
                print(f'  CAN monitor failed: {e}')

    def sample(self) -> dict:
        """Read and publish every bus; returns {bus: CANStatus} for the ones that could be read."""
        out = {}
        for name, port in self.buses.items():
            try:
                st = self._status(port)
            except Exception:  # noqa - a bus that cannot be read is skipped, the others still are
                continue
            out[name] = st
            p = self.pubs[name]
            p['utilization'].set(float(st.percent_bus_utilization))
            p['bus_off_count'].set(float(st.bus_off_count))
            p['tx_full_count'].set(float(st.tx_full_count))
            p['rx_errors'].set(float(st.receive_error_count))
            p['tx_errors'].set(float(st.transmit_error_count))
            last = self._last.get(name)
            if last is not None:
                if st.bus_off_count > last[0]:
                    print(f"*** CAN bus {name} went bus-off: count {last[0]} -> {st.bus_off_count} at {wpilib.Timer.get_timestamp():.1f}s ***")
                if st.tx_full_count > last[1]:
                    print(f"*** CAN bus {name} TX queue full: count {last[1]} -> {st.tx_full_count} at {wpilib.Timer.get_timestamp():.1f}s ***")
            self._last[name] = (st.bus_off_count, st.tx_full_count)
        pct = self._phoenix()
        if pct is not None:
            self.phoenix_pub.set(pct)
        return out

    def _phoenix_pct(self):
        """Phoenix's utilization of the Kraken bus in percent, or None (Phoenix not installed / cannot read it)."""
        try:
            if self._phoenix_bus is None:
                from phoenix6 import CANBus
                self._phoenix_bus = CANBus.systemcore(0) if hasattr(CANBus, 'systemcore') else CANBus('can_s0')
            return float(self._phoenix_bus.get_status().bus_utilization) * 100.0
        except Exception:  # noqa
            return None
