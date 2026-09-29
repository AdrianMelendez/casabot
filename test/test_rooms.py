"""Room segmentation. No ROS: `python3 test/test_rooms.py`."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from casabot.rooms import segment_rooms  # noqa: E402

RES = 0.05


def two_rooms(door_m):
    """4 x 3 m and 3 x 3 m rooms side by side, a door of door_m in the shared wall."""
    g = np.zeros((62, 144), dtype=np.int8)              # 3.1 x 7.2 m, row 0 = bottom
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    g[:, 81] = 100                                      # wall at x = 4.05 m
    half = int(door_m / RES / 2)
    g[31 - half:31 + half, 81] = 0                      # door in the middle
    return g


def test_doorway_separates_rooms():
    labels, rooms = segment_rooms(two_rooms(0.9), RES)
    assert len(rooms) == 2
    (r1, c1, a1), (r2, c2, a2) = rooms
    assert a1 > a2                                      # room_1 is the bigger one
    assert c1 < 81 < c2                                 # each place on its own side
    assert labels[31, 20] != labels[31, 120]


def test_open_plan_is_one_room():
    _, rooms = segment_rooms(two_rooms(2.5), RES)
    assert len(rooms) == 1


def test_place_is_the_most_open_spot():
    _, rooms = segment_rooms(two_rooms(0.9), RES)
    row, col, _ = rooms[0]
    # In a 4 x 3 m room the most open spots are on the centre line, 1.5 m from
    # the long walls; any of them is fine, but it must not hug a wall.
    wall_dist = min(row, 61 - row, col, 81 - col) * RES
    assert wall_dist >= 1.4


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all room checks passed')
