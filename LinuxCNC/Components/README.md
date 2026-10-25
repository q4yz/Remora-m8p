LinuxCNC components



Install every component in one go (install.sh already does this once):



&#x20;   sudo bash scripts/install\_hal\_components.sh             # from the repo root

&#x20;   sudo bash scripts/install\_hal\_components.sh remora-spi  # only some



It installs the build tools (`linuxcnc-uspace-dev`, `libcap-dev`) if missing, compiles each module with `halcompile --install` into LinuxCNC's module directory, and prints which ones failed. Re-run it whenever a component's source changes. Updates run without root and can't do it; the update screen says when it's needed.



Or by hand, from this folder:





sudo halcompile --install ./Remora-eth/remora-eth-3.0.c



sudo halcompile --install ./Remora-spi/remora-spi.c

`remora-spi` runs on the Raspberry Pi and, through Linux spidev, on other boards such as the BigTreeTech CB1, see [Remora-spi/README.md](Remora-spi/README.md).



sudo halcompile --install ./PRUencoder/PRUencoder.c



sudo halcompile --install ./NVMPG/nvmpg.c



sudo halcompile --install ./Remora/remora\_lpc.c



sudo halcompile --install ./PIDcontroller/PIDcontroller.c





Userspace Python components are not compiled — load them from HAL with `loadusr`, e.g. the Raspberry Pi temperature/load monitor:



&#x20;   loadusr -Wn rpi-monitor python3 ./RPiMonitor/rpi\_monitor.py



See \[RPiMonitor/readme.md](RPiMonitor/readme.md).



