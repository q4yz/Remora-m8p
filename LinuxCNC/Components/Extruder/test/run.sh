#!/bin/sh
# PC test of extruder.comp's flow / pressure-advance stage: its FUNCTION
# body, compiled against plain variables for its pins, drives a simulated
# step generator with a hard speed limit (a CAN node: 20 kHz).
# Linux / WSL with gcc:  sh LinuxCNC/Components/Extruder/test/run.sh
cd "$(dirname "$0")"
B=$(mktemp -d)
# the component's function: everything after "FUNCTION(_) {", its own closing brace included
awk 'f { print } /^FUNCTION\(_\) \{/ { f = 1 }' ../extruder.comp > "$B"/body.inc
gcc -std=c99 -O1 -g -Wall -Wno-unused-variable -I"$B" test.c -lm -o "$B"/test || { rm -rf "$B"; exit 1; }
"$B"/test; r=$?
rm -rf "$B"
echo "ALL: $([ $r = 0 ] && echo PASSED || echo FAILED)"
exit $r
