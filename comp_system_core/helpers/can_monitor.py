"""
Once a second, read the state of every CAN bus and publish it, so it lands in the .wpilog and Scope2429 can plot it.

WHY THIS EXISTS
---------------
Phoenix printed the CAN utilization of the Kraken bus ONCE, at start-up, when nothing was driving (0.0 %), and nothing
else about the buses was logged.  So after a match with lag, dropped controllers or brownout-like resets there was no
record of whether a bus was saturated, had gone bus-off, or was overflowing its transmit queue.

WHERE THE NUMBERS COME FROM
---------------------------
Two readers exist and each bus uses the first that works:

  1. wpilib.RobotController.get_can_status(port): utilization, bus-off, TX-full, receive and transmit errors.
  2. Phoenix: CANBus.systemcore(n).get_status(): bus_utilization, bus_off_count, tx_full_count, rec, tec.

On the competition SystemCore (2026-10-25) the WPILib call raised for all three buses, and nobody saw it because the monitor
swallowed the error.  Phoenix worked.  So the monitor now says, once per bus, which reader it uses and why the other failed
(it is in the console log), and publishes that as a `source` string.  WPILib is tried first because it needs no vendor library.

The three SystemCore buses (constants.k_can_bus_*): can_s0 = swerve drive Krakens, can_s1 = swerve turn SparkFlexes,
can_s2 = shooter, intake, PDH.  Published as NetworkTables values under constants.can_prefix (they go into the .wpilog):

    <can_prefix>/can_s0/utilization       percent busy, whichever reader gave it (see the unit note below)
    <can_prefix>/can_s0/bus_off_count      <- these four are running totals since boot; a rise is the event
    <can_prefix>/can_s0/tx_full_count
    <can_prefix>/can_s0/rx_errors
    <can_prefix>/can_s0/tx_errors
    <can_prefix>/can_s0/source             "wpilib", "phoenix" or "none"
    <can_prefix>/can_s0/phoenix_utilization_raw   Phoenix's number for the drive bus exactly as it returns it, for comparison

UNIT.  Phoenix documents bus_utilization as 0..1, but on the robot it returned about 2.45 on the Kraken bus, which cannot be a
fraction, so it is taken to be percent and published unchanged.  WPILib's header does not say.  Neither is confirmed against
a bus analyzer: treat the value as "percent, approximately" and use it to see shape and spikes, not to the decimal.

A rise in bus-off or TX-full is printed (so it is in the console log too).  Utilization is not printed.

Never raises: a diagnostic must not be able to stop the robot.
"""

from types import SimpleNamespace

import ntcore
import wpilib

import constants


class CanMonitor:
    def __init__(self, period_loops: int = 50, prefix: str = constants.can_prefix, status_fn=None, phoenix_status_fn=None) -> None:
        self.period = period_loops
        self.counter = 0
        self.buses = {'can_s0': constants.k_can_bus_drive, 'can_s1': constants.k_can_bus_turn, 'can_s2': constants.k_can_bus_other}
        self._wpilib = status_fn or wpilib.RobotController.get_can_status      # injectable for tests
        self._phoenix = phoenix_status_fn or self._phoenix_status
        self._phoenix_buses = {}
        self._last = {}                                                         # bus -> (bus_off, tx_full) at the last sample
        self._source = {}                                                       # bus -> reader in use, so a change is announced once
        self._warned = set()
        inst = ntcore.NetworkTableInstance.get_default()
        self.pubs = {name: {k: inst.get_double_topic(f'{prefix}/{name}/{k}').publish()
                            for k in ('utilization', 'bus_off_count', 'tx_full_count', 'rx_errors', 'tx_errors')}
                     for name in self.buses}
        self.source_pubs = {name: inst.get_string_topic(f'{prefix}/{name}/source').publish() for name in self.buses}
        self.phoenix_pub = inst.get_double_topic(f'{prefix}/can_s0/phoenix_utilization_raw').publish()

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
        """Read and publish every bus; returns {bus: reading} for the ones that could be read."""
        out = {}
        for name, port in self.buses.items():
            st, source, errors = None, 'none', []
            try:
                st, source = self._wpilib(port), 'wpilib'
            except Exception as e:  # noqa
                errors.append(f'WPILib: {e!r}')
            if st is None:
                try:
                    ph = self._phoenix(name)
                    st = SimpleNamespace(percent_bus_utilization=ph.bus_utilization, bus_off_count=ph.bus_off_count,
                                         tx_full_count=ph.tx_full_count, receive_error_count=ph.rec, transmit_error_count=ph.tec)
                    source = 'phoenix'
                except Exception as e:  # noqa
                    errors.append(f'Phoenix: {e!r}')
            self.source_pubs[name].set(source)
            if self._source.get(name) != source:                                # say it once: which reader, or why none
                self._source[name] = source
                if st is None:
                    print(f"  CAN monitor: NO reading for {name}  ({'; '.join(errors)})")
                else:
                    print(f"  CAN monitor: {name} read from {source}" + (f"  ({'; '.join(errors)})" if errors else ''))
            if st is None:
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
        try:                                                                    # the drive bus as Phoenix reports it, for comparison
            self.phoenix_pub.set(float(self._phoenix('can_s0').bus_utilization))
        except Exception:  # noqa
            pass
        return out

    def _phoenix_status(self, name: str):
        """Phoenix's CANBusStatus for 'can_s0'..'can_s4'."""
        if name not in self._phoenix_buses:
            from phoenix6 import CANBus
            self._phoenix_buses[name] = CANBus.systemcore(int(name[-1]))
        return self._phoenix_buses[name].get_status()
