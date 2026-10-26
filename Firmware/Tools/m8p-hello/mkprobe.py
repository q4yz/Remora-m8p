#!/usr/bin/env python3
"""Builds m8p_probe.bin from the single probe copy m8p_probe_blob.bin (linked at 0x08010000).

File layout: vector (SP, PC = 0x08010001) at offset 0, and one probe copy at every offset
0x1000 ... 0x10000. With a bootloader of B bytes the copy at file offset 0x10000 - B ends up
at 0x08010000, which is where the vector points. The copy finds its own offset in the ID word
at +0x0FFC and reports the bootloader size.
"""
import struct
import sys

PROBE_ADDR = 0x08010000
STEP = 0x1000
ID_OFFSET = 0x0FFC

blob = open(sys.argv[1] if len(sys.argv) > 1 else 'm8p_probe_blob.bin', 'rb').read()
if len(blob) > ID_OFFSET:
    sys.exit(f'probe copy is {len(blob)} bytes, at most {ID_OFFSET} fit')

image = bytearray(b'\xff' * (0x10000 + STEP))
image[0:8] = struct.pack('<II', 0x20024000, PROBE_ADDR | 1)

for offset in range(STEP, 0x10000 + STEP, STEP):
    image[offset:offset + len(blob)] = blob
    image[offset + ID_OFFSET:offset + ID_OFFSET + 4] = struct.pack('<I', offset)

out = sys.argv[2] if len(sys.argv) > 2 else 'm8p_probe.bin'
open(out, 'wb').write(image)
print(f'{out}: {len(image)} bytes, {len(blob)} byte probe at 16 offsets, '
      f'bootloader sizes 0 ... {(0x10000 - STEP) // 1024} KB')
