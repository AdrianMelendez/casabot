"""Frontier detection and map saving. numpy/scipy only, no ROS, so it tests anywhere.

A frontier is known-free space touching unknown space: the edge of what the
robot has seen. Driving to frontiers until none are left maps the whole place.
"""

import os

import numpy as np
from scipy import ndimage

FREE, UNKNOWN = 0, -1


def find_frontiers(grid, min_cells, clearance_cells=0):
    """Frontier clusters in an occupancy grid (row 0 = lowest y, as in ROS).

    grid: 2D int array with -1 unknown, 0 free, 1-100 occupied.
    Returns [(row, col, size)], one per cluster of at least min_cells cells,
    largest first. (row, col) is the cluster cell nearest its centroid among
    those at least clearance_cells from any occupied cell, so it is a free cell
    the robot can be sent to without wedging itself against furniture. A
    cluster with no such cell is skipped: it is a pocket, not a way forward.
    """
    free = grid == FREE
    unknown = grid == UNKNOWN
    touches_unknown = np.zeros_like(free)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & touches_unknown

    # Distance from every cell to the nearest occupied cell, in cells.
    clearance = ndimage.distance_transform_edt(grid < 50)

    labels, count = ndimage.label(frontier, structure=np.ones((3, 3)))
    clusters = []
    for i in range(1, count + 1):
        rows, cols = np.nonzero(labels == i)
        size = len(rows)
        if size < min_cells:
            continue
        roomy = clearance[rows, cols] >= clearance_cells
        if not roomy.any():
            continue
        cr, cc = rows.mean(), cols.mean()
        rows, cols = rows[roomy], cols[roomy]
        k = np.argmin((rows - cr) ** 2 + (cols - cc) ** 2)
        clusters.append((int(rows[k]), int(cols[k]), size))
    return sorted(clusters, key=lambda c: -c[2])


def save_map(grid, resolution, origin_xy, path):
    """Write grid as path.pgm + path.yaml in the format map_server loads.

    Same thresholds as map_saver_cli: occupied >= 65, free <= 19, else unknown.
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[(grid >= 0) & (grid <= 19)] = 254
    img[grid >= 65] = 0
    img = img[::-1]  # image row 0 is the top of the map
    h, w = img.shape
    with open(path + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    with open(path + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(path)}.pgm\n'
                f'mode: trinary\n'
                f'resolution: {resolution}\n'
                f'origin: [{origin_xy[0]}, {origin_xy[1]}, 0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.196\n')
