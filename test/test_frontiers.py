"""Frontier detection and map saving. No ROS: `python3 test/test_frontiers.py`."""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from casabot.frontiers import find_frontiers, save_map  # noqa: E402


def room():
    """10x10: known free left half, unknown right half, a wall along the top."""
    g = np.full((10, 10), -1, dtype=np.int8)
    g[:, :5] = 0
    g[9, :] = 100
    return g


def test_frontier_is_the_free_unknown_edge():
    (row, col, size), = find_frontiers(room(), min_cells=3)
    assert col == 4                  # last free column, next to the unknown
    assert size == 9                 # rows 0-8; row 9 is wall, not frontier
    assert room()[row, col] == 0     # the goal is always a free cell


def test_small_frontiers_are_ignored():
    g = np.zeros((10, 10), dtype=np.int8)
    g[5, 5] = -1                     # one unknown cell: a sensor gap, not a room
    assert find_frontiers(g, min_cells=5) == []


def test_goal_keeps_clear_of_furniture():
    g = room()
    g[:4, 3] = 100                   # furniture right next to the lower frontier
    (row, col, _), = find_frontiers(g, min_cells=3, clearance_cells=2)
    assert row >= 5                  # picked from the roomy part of the edge
    g[:, 3] = 100                    # now the whole edge is hemmed in
    assert find_frontiers(g, min_cells=3, clearance_cells=2) == []


def test_fully_explored_map_has_no_frontiers():
    g = np.zeros((10, 10), dtype=np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    assert find_frontiers(g, min_cells=1) == []


def test_save_map_flips_rows_and_thresholds():
    g = np.array([[0, 100], [-1, 50]], dtype=np.int8)   # row 0 = bottom
    with tempfile.TemporaryDirectory() as d:
        save_map(g, 0.05, (1.0, 2.0), os.path.join(d, 'm'))
        raw = open(os.path.join(d, 'm.pgm'), 'rb').read()
        assert raw.startswith(b'P5\n2 2\n255\n')
        pixels = list(raw[-4:])
        assert pixels == [205, 205, 254, 0]   # top row first: unknown, 50 -> unknown
        assert 'origin: [1.0, 2.0, 0]' in open(os.path.join(d, 'm.yaml')).read()


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all frontier checks passed')
