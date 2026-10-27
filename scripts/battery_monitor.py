#!/usr/bin/env python3
"""Battery monitor: voltage, current, power, charge (%), energy used and battery health of the 12 V motor battery
(3S Li-ion pack through the BMS; the Pi has its own power bank and is not measured).

Sources (parameter `source`):
  sim     no sensor yet: a battery model driven by the drive commands (/diff_cont/cmd_vel_unstamped, the output of
          twist_mux). Clearly marked "sim" everywhere, so its numbers are never mistaken for measurements.
  ina219  INA219 board on the Pi's I2C bus (default address 0x40, shunt R100 = 0.1 ohm, up to 3.2 A)
  ina226  INA226 board (shunt R010 = 0.01 ohm recommended; the common R100 boards only measure up to ~0.8 A)

Charge (%): from the pack voltage, corrected for the drop under load (voltage + current x internal resistance),
looked up in a Li-ion resting-voltage table, then smoothed, so it does not jump every time the motors start.
Health: the internal resistance is measured whenever the current changes quickly (it is the voltage change divided
by the current change); it rises as cells age or when one cell is weak (like the 1.2 V cell found on 2026-10-23).

Publishes /battery_status (String JSON, latched) every `period` s:
  {source, v, a, w, pct, wh, state: ok|low|critical|charging|off, r_ohm, over_range, t}
  off = no voltage (below `off_volts`): the power switch is on OFF or CHARGE (the sensor sits on the RUN side, so
  it cannot see the charging itself); pct is then null and is read fresh once the switch is back on RUN.
Simulation only: /battery_sim_set (std_msgs/Float32, %) sets the simulated charge, e.g. 100 = "charged".
  wh = energy taken from the battery since this program started (event_logger turns it into Wh per delivery).

  python3 src/my_bot/scripts/battery_monitor.py --ros-args -p source:=sim            (laptop, before the sensor)
  python3 src/my_bot/scripts/battery_monitor.py --ros-args -p source:=ina219         (Pi, after fitting it)
"""
import json
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

# resting voltage of one Li-ion cell -> charge %, typical 18650 (NMC) curve
OCV = [(3.00, 0), (3.30, 5), (3.50, 10), (3.60, 18), (3.70, 33), (3.75, 44), (3.80, 54), (3.85, 62),
       (3.90, 70), (3.95, 76), (4.00, 82), (4.10, 92), (4.20, 100)]


def pct_from_cell(v):
    if v <= OCV[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(OCV, OCV[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


def cell_from_pct(p):
    for (v0, p0), (v1, p1) in zip(OCV, OCV[1:]):
        if p <= p1:
            return v0 + (v1 - v0) * (p - p0) / (p1 - p0)
    return OCV[-1][0]


class Ina:
    """INA219 / INA226 read with plain register access (no extra driver library)."""

    def __init__(self, kind, bus, address, shunt_ohm):
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus          # python3-smbus from apt
        self.bus, self.addr, self.kind, self.shunt = SMBus(bus), address, kind, shunt_ohm
        # INA219: 32 V range, +-320 mV shunt, 128-sample averaging, continuous; INA226: 64 samples, 1.1 ms, continuous
        self._write(0x00, 0x3FFF if kind == 'ina219' else 0x4727)

    def _write(self, reg, value):
        self.bus.write_word_data(self.addr, reg, ((value & 0xFF) << 8) | (value >> 8))   # the chip is big-endian

    def _read(self, reg):
        raw = self.bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def read(self):
        """(volts, amps, over_range)"""
        shunt = self._read(0x01)
        shunt = shunt - 65536 if shunt > 32767 else shunt
        bus = self._read(0x02)
        if self.kind == 'ina219':
            return (bus >> 3) * 0.004, shunt * 10e-6 / self.shunt, bool(bus & 1)    # OVF bit: current too high
        return bus * 0.00125, shunt * 2.5e-6 / self.shunt, abs(shunt) >= 32767


class SimBattery:
    """A 3S pack drained by the motors: idle draw + a share per m/s and per rad/s of the drive command."""

    def __init__(self, node, cells, capacity_ah, r_ohm, start_pct, speedup):
        self.cells, self.cap, self.r, self.speedup = cells, capacity_ah, r_ohm, speedup
        self.soc = start_pct
        self.cmd, self.cmd_t = (0.0, 0.0), 0.0
        node.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self._on_cmd, 10)
        node.create_subscription(Float32, '/battery_sim_set', self._on_set, 10)
        self.last = time.monotonic()

    def _on_set(self, msg):
        self.soc = max(0.0, min(100.0, msg.data))

    def _on_cmd(self, msg):
        self.cmd, self.cmd_t = (msg.linear.x, msg.angular.z), time.monotonic()

    def read(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        v, w = self.cmd if now - self.cmd_t < 0.5 else (0.0, 0.0)     # twist_mux stops sending when idle
        amps = 0.25 + 3.5 * min(abs(v), 0.3) + 0.9 * min(abs(w), 1.5)     # ~1.3 A cruising, ~1.6 A turning on the spot
        self.soc = max(0.0, self.soc - amps * dt * self.speedup / 3600.0 / self.cap * 100.0)
        return self.cells * cell_from_pct(self.soc) - amps * self.r, amps, False


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        p = self.declare_parameter
        p('source', 'sim')                 # sim | ina219 | ina226
        p('i2c_bus', 1)
        p('address', 0x40)
        p('shunt_ohm', 0.1)                # R100 = 0.1 (INA219 boards); R010 = 0.01
        p('cells', 3)
        p('capacity_ah', 2.5)              # 18650 2500 mAh cells in series
        p('internal_ohm', 0.15)            # whole pack incl. BMS and wires: first guess, replaced by the measurement
        p('low_pct', 25.0)                 # below: no new deliveries (delivery_manager)
        p('critical_pct', 10.0)            # below: go home now
        p('off_volts', 6.0)                # below: power switched off (a 3S pack is never this low while on)
        p('period', 1.0)
        p('sim_start_pct', 85.0)
        p('sim_speedup', 1.0)              # >1 drains the simulated battery faster (testing)
        g = lambda n: self.get_parameter(n).value
        self.source, self.cells = g('source'), g('cells')
        self.r_default = g('internal_ohm')
        self.low, self.critical, self.off_volts = g('low_pct'), g('critical_pct'), g('off_volts')
        if self.source == 'sim':
            self.dev = SimBattery(self, self.cells, g('capacity_ah'), self.r_default, g('sim_start_pct'),
                                  g('sim_speedup'))
        else:
            self.dev = Ina(self.source, g('i2c_bus'), g('address'), g('shunt_ohm'))
        self.pub = self.create_publisher(String, '/battery_status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pct = None
        self.wh = 0.0
        self.last_t = None
        self.prev = None                   # (volts, amps) of the previous reading, for the resistance
        self.r_samples = []
        self.state = None
        self.fails = 0
        self.create_timer(g('period'), self._tick)
        self.get_logger().info('Battery monitor: source %s' % self.source)

    def _resistance(self):
        return statistics.median(self.r_samples) if len(self.r_samples) >= 5 else self.r_default

    def _tick(self):
        try:
            volts, amps, over = self.dev.read()
            self.fails = 0
        except OSError as exc:              # loose I2C wire: say so instead of showing old numbers
            self.fails += 1
            if self.fails in (1, 10) or self.fails % 60 == 0:
                self.get_logger().warn('Battery sensor not answering: %s' % exc)
            self.pub.publish(String(data=json.dumps({'source': self.source, 'error': 'sensor not answering',
                                                     't': round(time.time(), 1)})))
            return
        now = time.monotonic()
        if volts < self.off_volts:
            self.pct, self.prev, self.last_t = None, None, None      # start fresh when it is switched on again
            self._report('off', volts, amps, None, over)
            return
        if self.last_t is not None:
            self.wh += max(volts * amps, 0.0) * (now - self.last_t) / 3600.0
        self.last_t = now
        if self.prev is not None and abs(amps - self.prev[1]) > 0.4:      # load step: R = -dV / dI
            r = -(volts - self.prev[0]) / (amps - self.prev[1])
            if 0.02 < r < 2.0:
                self.r_samples = (self.r_samples + [r])[-30:]
        self.prev = (volts, amps)
        r = self._resistance()
        raw = pct_from_cell((volts + max(amps, 0.0) * r) / self.cells)
        # smooth (~20 s); the first reading, and a jump of over 20 % (battery changed or charged), taken as is
        self.pct = raw if self.pct is None or abs(raw - self.pct) > 20 else self.pct + 0.05 * (raw - self.pct)
        if amps < -0.05:
            state = 'charging'
        elif self.pct < self.critical:
            state = 'critical'
        elif self.pct < self.low:
            state = 'low'
        else:
            state = 'ok'
        self._report(state, volts, amps, self.pct, over)

    def _report(self, state, volts, amps, pct, over):
        if state != self.state:
            self.get_logger().info('Battery %s: %s %%, %.2f V, %.2f A' % (state, 'unknown' if pct is None else
                                                                          '%.0f' % pct, volts, amps))
            self.state = state
        self.pub.publish(String(data=json.dumps({
            'source': self.source, 'v': round(volts, 2), 'a': round(amps, 2), 'w': round(volts * amps, 1),
            'pct': None if pct is None else round(pct), 'wh': round(self.wh, 3), 'state': state,
            'r_ohm': round(self._resistance(), 3) if len(self.r_samples) >= 5 else None, 'over_range': over,
            't': round(time.time(), 1)})))


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
