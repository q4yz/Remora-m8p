#!/bin/sh
# PC test of the CAN hub / node protocol logic (CanHub.cpp, CanNode.cpp) on a simulated bus.
# Linux / WSL with g++:  sh Firmware/Tests/can/run.sh
cd "$(dirname "$0")"
D=../../FirmwareSource/Remora-OS6/TARGET_STM32G0/drivers/can
B=$(mktemp -d)
# the sources include "RemoraCAN.h" from their own folder, build copies next to the simulated bus
cp $D/CanHub.* $D/CanNode.* $D/canProtocol.h stubs/RemoraCAN.h "$B"/
g++ -std=c++14 -O1 -g -Wall -Wextra -Istubs -I"$B" -include vector test.cpp "$B"/CanHub.cpp "$B"/CanNode.cpp -o "$B"/test || exit 1
r=0
for s in uuid assign two-nodes unknown lost node-reboot bad-config; do "$B"/test $s || r=1; done
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
