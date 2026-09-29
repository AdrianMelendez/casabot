#!/usr/bin/env python3
"""Talk to the base ESP32 directly, without ROS. For bring-up (docs/bringup.md).

    python3 tools/base_check.py /dev/ttyUSB1 listen [seconds]
    python3 tools/base_check.py /dev/ttyUSB1 spin LEFT RIGHT [seconds]     # rad/s

listen: telemetry rates, encoder ticks and the IMU, with no motion commanded.
spin:   drive the wheels at LEFT/RIGHT rad/s, re-sending the command every
        0.1 s (the firmware stops the motors 0.5 s after the last one), then
        compare the speed the encoders measured with what was asked for.
"""

import math
import sys
import time

import serial

TICKS_PER_REV = 330.0        # must match the firmware; see its TICKS_PER_REV comment


def main(argv):
    if len(argv) < 3 or argv[2] not in ('listen', 'spin'):
        raise SystemExit(__doc__)
    port, mode = argv[1], argv[2]
    if mode == 'spin':
        left, right = float(argv[3]), float(argv[4])
        seconds = float(argv[5]) if len(argv) > 5 else 3.0
    else:
        left = right = None
        seconds = float(argv[3]) if len(argv) > 3 else 5.0

    link = serial.Serial(port, 115200, timeout=0.05)
    start = time.monotonic()
    sent = 0.0
    counts = {'o': 0, 'i': 0}
    first = last = None
    imu = None
    while time.monotonic() - start < seconds:
        now = time.monotonic()
        if left is not None and now - sent >= 0.1:
            link.write(f'v {left:.4f} {right:.4f}\n'.encode())
            sent = now
        fields = link.readline().decode('ascii', errors='ignore').split()
        if len(fields) == 3 and fields[0] == 'o':
            counts['o'] += 1
            ticks = (int(fields[1]), int(fields[2]))
            # Skip the spin-up half second when measuring speed.
            if first is None and now - start > min(0.5, seconds / 2):
                first = (now, ticks)
            last = (now, ticks)
        elif len(fields) == 7 and fields[0] == 'i':
            counts['i'] += 1
            imu = [float(v) for v in fields[1:]]
    if left is not None:
        link.write(b'v 0.0000 0.0000\n')

    print(f"telemetry: {counts['o'] / seconds:.0f} odometry and {counts['i'] / seconds:.0f} IMU "
          f"lines per second (firmware sends 50 and 50)")
    if last is None:
        raise SystemExit('no `o` lines at all: wrong port, wrong baud, or the sketch is not running')
    print(f'encoder ticks now: left {last[1][0]}, right {last[1][1]}')
    if imu:
        ax, ay, az, gx, gy, gz = imu
        print(f'IMU: az = {az:+.2f} m/s2 (about +9.8 lying flat, component side up), '
              f'gz = {gz:+.3f} rad/s')
    if left is not None and first is not None and last[0] > first[0]:
        span = last[0] - first[0]
        rate = [2 * math.pi * (last[1][k] - first[1][k]) / TICKS_PER_REV / span for k in (0, 1)]
        for side, asked, got in (('left', left, rate[0]), ('right', right, rate[1])):
            verdict = 'ok' if abs(got - asked) <= 0.15 * abs(asked) + 0.1 else 'CHECK'
            print(f'{side:5s} wheel: asked {asked:+.2f} rad/s, measured {got:+.2f} rad/s  {verdict}')


if __name__ == '__main__':
    main(sys.argv)
