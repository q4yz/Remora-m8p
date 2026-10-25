## NAME

rpi_monitor.py

## SYNOPSIS

**loadusr -Wn rpi-monitor** python3 /path/to/Components/RPiMonitor/rpi_monitor.py *[interval_s]*

## DESCRIPTION

A userspace HAL component that publishes the Raspberry Pi's SoC temperature and CPU load as HAL pins, so they can be watched in halmeter/halscope, compared against a limit with a `comp`, or wired anywhere else in HAL. Pure Python standard library plus LinuxCNC's own `hal` module — nothing to install.

The optional argument is the sampling interval in seconds (default `1.0`).

### PINS

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `rpi-monitor.cpu-temp` | float | OUT | SoC temperature in °C (`/sys/class/thermal/thermal_zone0/temp`) |
| `rpi-monitor.cpu-load` | float | OUT | CPU busy time across all cores, 0–100 % (delta of `/proc/stat` between samples; iowait counts as idle) |
| `rpi-monitor.ok` | bit | OUT | TRUE while the last sample read cleanly |

If a read fails (e.g. the sensor file doesn't exist because this isn't a Pi) the value pins **keep their last good value** and `ok` goes FALSE, instead of publishing a misleading 0. Gate any alarm logic on `ok`.

### EXAMPLE

In `custom.hal` (or any HAL file):

```
loadusr -Wn rpi-monitor python3 /home/pi/LinuxCNC_UI/Components/RPiMonitor/rpi_monitor.py 1.0

net cpu-temp rpi-monitor.cpu-temp
net cpu-load rpi-monitor.cpu-load
net cpu-mon-ok rpi-monitor.ok
```

Over-temperature alarm (Pi 4 throttles at 80 °C):

```
loadrt comp names=cpu-hot
addf cpu-hot servo-thread
net cpu-temp => cpu-hot.in0
setp cpu-hot.in1 75
net cpu-too-hot <= cpu-hot.out
```

`-W` makes HAL wait until the component's pins exist before the `net` lines run.

### TESTING OFF-PI

Two environment variables point the component at other files:

* `RPI_MONITOR_TEMP_FILE` — default `/sys/class/thermal/thermal_zone0/temp` (millidegrees)
* `RPI_MONITOR_STAT_FILE` — default `/proc/stat`

### LIMITS

* Reports the aggregate of all four cores — a single saturated core on a Pi 4 reads as ~25 %.
* Load is averaged over one sampling interval, so it's smooth rather than instantaneous.
* Not wired into the web UI; the pins are for HAL-side monitoring.
