#!/bin/sh
# PC test of the event driven step generation (drivers/stepScheduler) on a simulated timer.
# Linux / WSL with g++:  sh Firmware/Tests/stepsched/run.sh
cd "$(dirname "$0")"
S=../../FirmwareSource/Remora-OS6/drivers/stepScheduler
B=$(mktemp -d)
g++ -std=c++14 -O1 -g -Wall -Wextra -I$S test.cpp $S/StepScheduler.cpp -o "$B"/test || { rm -rf "$B"; exit 1; }
"$B"/test; r=$?
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
