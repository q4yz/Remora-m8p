#!/usr/bin/env python3
"""Raspberry Pi health monitor — a userspace LinuxCNC HAL component.

Samples the SoC temperature and overall CPU load and publishes them as
HAL pins, so they can be watched in halmeter/halscope, latched into a
`comp` against an over-temperature limit, or wired to the UI.

    loadusr -Wn rpi-monitor python3 /path/to/rpi_monitor.py [interval_s]

Pins (component name: ``rpi-monitor``):

    rpi-monitor.cpu-temp   float OUT  SoC temperature, degrees Celsius
    rpi-monitor.cpu-load   float OUT  CPU busy time across all cores, 0-100 %
    rpi-monitor.ok         bit   OUT  TRUE while the last sample read cleanly

If a sample fails (sensor file missing, e.g. not running on a Pi) the
value pins keep their last good value and ``ok`` goes FALSE, rather
than publishing a misleading 0.

Environment overrides (mainly for testing off-Pi):

    RPI_MONITOR_TEMP_FILE   default /sys/class/thermal/thermal_zone0/temp
    RPI_MONITOR_STAT_FILE   default /proc/stat

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2 of the License, or (at your
option) any later version.
"""

import os
import signal
import sys
import time

DEFAULT_INTERVAL_S = 1.0
TEMP_FILE = os.environ.get("RPI_MONITOR_TEMP_FILE", "/sys/class/thermal/thermal_zone0/temp")
STAT_FILE = os.environ.get("RPI_MONITOR_STAT_FILE", "/proc/stat")


def read_cpu_temp(path=TEMP_FILE):
    """SoC temperature in °C. The kernel reports millidegrees."""
    with open(path) as f:
        return int(f.read().strip()) / 1000.0


def read_cpu_times(path=STAT_FILE):
    """Return ``(busy, total)`` jiffies from the aggregate ``cpu`` line.

    Fields: user nice system idle iowait irq softirq steal ...
    iowait counts as idle — the CPU wasn't executing anything.
    """
    with open(path) as f:
        fields = f.readline().split()
    if not fields or fields[0] != "cpu":
        raise ValueError("unexpected %s format" % path)
    values = [int(v) for v in fields[1:9]]
    idle = values[3] + values[4]
    total = sum(values)
    return total - idle, total


class LoadSampler:
    """CPU load is a rate, so it needs two samples: percent busy since the last call."""

    def __init__(self, path=STAT_FILE):
        self._path = path
        self._last = read_cpu_times(path)

    def sample(self):
        busy, total = read_cpu_times(self._path)
        last_busy, last_total = self._last
        self._last = (busy, total)
        d_total = total - last_total
        if d_total <= 0:
            return 0.0
        return max(0.0, min(100.0, 100.0 * (busy - last_busy) / d_total))


def main(argv):
    import hal  # only present inside a LinuxCNC install — imported late so the readers above are testable anywhere

    interval = float(argv[1]) if len(argv) > 1 else DEFAULT_INTERVAL_S
    if interval <= 0:
        print("rpi-monitor: interval must be > 0", file=sys.stderr)
        return 1

    h = hal.component("rpi-monitor")
    h.newpin("cpu-temp", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("cpu-load", hal.HAL_FLOAT, hal.HAL_OUT)
    h.newpin("ok", hal.HAL_BIT, hal.HAL_OUT)

    # `loadusr` stops us with SIGTERM; turn it into a normal exit so HAL cleans up.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    try:
        sampler = LoadSampler()
    except (OSError, ValueError) as exc:
        print("rpi-monitor: cannot read %s: %s" % (STAT_FILE, exc), file=sys.stderr)
        sampler = None
    h["ok"] = False
    h.ready()

    try:
        while True:
            ok = True
            try:
                h["cpu-temp"] = read_cpu_temp()
            except (OSError, ValueError):
                ok = False
            try:
                if sampler is None:
                    sampler = LoadSampler()
                h["cpu-load"] = sampler.sample()
            except (OSError, ValueError):
                ok = False
            h["ok"] = ok
            time.sleep(interval)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        h.exit()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
