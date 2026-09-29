"""Split a finished map into rooms. numpy/scipy only, no ROS.

Doorways are where free space gets narrow. Cells far from every obstacle form
separate blobs, one per room, because the narrow doorways cut them apart. Each
blob is then grown back out through free space until the rooms meet in the
doorways. A corridor is too narrow to seed a room, so it is shared out among
the rooms it connects. An opening wide enough to keep two cores connected
(a truly open-plan space) makes them one room.
"""

import numpy as np
from scipy import ndimage


def segment_rooms(grid, resolution, core_clearance=0.6, min_core_area=0.25):
    """Label the free cells of grid by room.

    grid: 2D int array, -1 unknown, 0 free, >= 50 occupied (row 0 = lowest y).
    core_clearance: metres from any obstacle a cell must be to seed a room.
        More than half a doorway (0.4-0.45 m) and more than half a corridor
        (0.55 m for 1.1 m), so neither seeds a room of its own or links two.
    min_core_area: m2 of seed below which it is clutter, not a room. The seed
        is only the open middle of a room, so this is far smaller than a room:
        a bathroom's core, between bath, sink and toilet, is under 1 m2.
    Returns (labels, rooms): labels is 0 outside free space and 1..n per room,
    rooms is [(row, col, area_m2)] largest first, where (row, col) is the most
    open cell of the room: the natural spot to send the robot.
    """
    free = grid == 0
    clearance = ndimage.distance_transform_edt(free) * resolution

    seeds, count = ndimage.label(clearance > core_clearance)
    min_cells = min_core_area / (resolution * resolution)
    sizes = ndimage.sum(np.ones_like(seeds), seeds, index=range(1, count + 1))
    for i, size in enumerate(sizes, start=1):
        if size < min_cells:
            seeds[seeds == i] = 0

    # Grow every seed through free space, one ring per step, until nothing
    # unlabelled and reachable is left. Rooms meet in the doorways.
    labels = seeds.copy()
    while True:
        grown = ndimage.grey_dilation(labels, size=(3, 3))
        new = free & (labels == 0) & (grown > 0)
        if not new.any():
            break
        labels[new] = grown[new]

    rooms = []
    for i in np.unique(labels[labels > 0]):
        inside = labels == i
        area = inside.sum() * resolution * resolution
        best = np.argmax(np.where(inside, clearance, -1))
        row, col = np.unravel_index(best, grid.shape)
        rooms.append((int(row), int(col), float(area)))
    rooms.sort(key=lambda r: -r[2])

    # Renumber so label k is rooms[k - 1]: room_1 is the biggest.
    relabel = np.zeros(labels.max() + 1, dtype=labels.dtype)
    for k, (row, col, _) in enumerate(rooms, start=1):
        relabel[labels[row, col]] = k
    return relabel[labels], rooms
