#!/bin/sh
# PC test of the CAN hub / node protocol logic (CanHub.cpp, CanNode.cpp) on a simulated bus.
# Linux / WSL with g++:  sh Firmware/Tests/can/run.sh
cd "$(dirname "$0")"
S=../../FirmwareSource/Remora-OS6
D=$S/drivers/can
B=$(mktemp -d)
# the logic only knows CanBus (drivers/can/CanBus.h): the simulated bus and pins are in stubs/
g++ -std=c++14 -O1 -g -Wall -Wextra -Istubs -I$D -I$S -include vector test.cpp $D/CanHub.cpp $D/CanNode.cpp \
    $D/NodeTmc2209.cpp stubs/canNodeIoSim.cpp -o "$B"/test || { rm -rf "$B"; exit 1; }
r=0
for s in uuid assign two-nodes unknown lost node-reboot bad-config \
         pins config-io hub-lost node-reboot-config rejected bit-conflict lossy \
         mandatory no-status-bit joint joint-reverse joint-hub-lost joint-config \
         joint-reboot tmc tmc-reset tmc-no-power tmc-write-lost tmc-config \
         thermistor thermistor-lost thermistor-config pwm pwm-lost pwm-config \
         rgb rgb-lost rgbw rgb-config safe-timeout safe-timeout-hub-lost safe-timeout-never; do "$B"/test $s || r=1; done
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
