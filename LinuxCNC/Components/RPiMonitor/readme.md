## NAME

rpi_monitor.py

## SYNOPSIS

**loadusr -Wn rpi-monitor** python3 /path/to/Components/RPiMonitor/rpi_monitor.py *[interval_s]*

## DESCRIPTION

A userspace HAL component that publishes the host board's SoC temperature and CPU load as HAL pins — on a Raspberry Pi or a BigTreeTech CB1 — so they can be watched in halmeter/halscope, compared against a limit with a `comp`, or wired anywhere else in HAL. Pure Python standard library plus LinuxCNC's own `hal` module — nothing to install.

The optional argument is the sampling interval in seconds (default `1.0`).

### PINS

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `rpi-monitor.cpu-temp` | float | OUT | SoC temperature in °C (the CPU's thermal zone, see below) |
| `rpi-monitor.cpu-load` | float | OUT | CPU busy time across all cores, 0–100 % (delta of `/proc/stat` between samples; iowait counts as idle) |
| `rpi-monitor.ok` | bit | OUT | TRUE while the last sample read cleanly |

If a read fails (e.g. the sensor file doesn't exist because this isn't a Pi) the value pins **keep their last good value** and `ok` goes FALSE, instead of publishing a misleading 0. Gate any alarm logic on `ok`.

### EXAMPLE

In `custom.hal` (or any HAL file):

```
loadusr -Wn rpi-monitor rpi-monitor 1.0

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

### SUPPORTED BOARDS

The temperature is read from the CPU's thermal zone under `/sys/class/thermal`, picked by its `type` rather than its number:

| Board | SoC | Zones | Used |
|---|---|---|---|
| Raspberry Pi 3 / 4 / 5 | Broadcom | `cpu-thermal` only | `cpu-thermal` |
| BTT CB1 | Allwinner H616 | `cpu-thermal`, `gpu-thermal`, `ve-thermal`, `ddr-thermal` (order depends on the kernel) | `cpu-thermal` |

Matched types, best first: `cpu-thermal`, `cpu_thermal`, `cpu0-thermal`, `soc-thermal`, `soc_thermal`. If none matches, `thermal_zone0` is used, as before. The component prints the file it reads on startup.

### TESTING OFF-PI

Two environment variables point the component at other files:

* `RPI_MONITOR_TEMP_FILE` — read this file (millidegrees) instead of looking for the CPU zone
* `RPI_MONITOR_THERMAL_DIR` — where to look for the zones, default `/sys/class/thermal`
* `RPI_MONITOR_STAT_FILE` — default `/proc/stat`

### LIMITS

* Reports the aggregate of all cores — a single saturated core on a 4-core Pi 4 or CB1 reads as ~25 %.
* Load is averaged over one sampling interval, so it's smooth rather than instantaneous.
* To show the temperature in the LinuxCNC web UI's temperature plot, add a sensor to the machine config (the UI wires the pin, it does not load the component):

  ```
  [temperature_sensor host]
  sensor_type: hal
  sensor_pin: rpi-monitor.cpu-temp
  ```
