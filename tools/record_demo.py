#!/usr/bin/env python3
"""Record an exploration run for tools/render_demo.py.

    python3 tools/record_demo.py /tmp/frames      # alongside sim + explore launches

Once per second of sim time, saves the occupancy grid, robot pose, Nav2's
current plan and the latest lidar scan (all in the map frame) to NNNNN.npz.
"""

import math
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node('demo_recorder')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    tf = Buffer()
    TransformListener(tf, n)
    st = {'map': None, 'plan': np.zeros((0, 2)), 'scan': None}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/map', lambda m: st.update(map=m), latched)
    n.create_subscription(Path, '/plan', lambda m: st.update(plan=np.array(
        [[p.pose.position.x, p.pose.position.y] for p in m.poses]).reshape(-1, 2)), 10)
    n.create_subscription(LaserScan, '/scan', lambda m: st.update(scan=m), 10)

    k, last = 0, -1.0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        now = n.get_clock().now().nanoseconds * 1e-9
        if now - last < 1.0 or st['map'] is None:
            continue
        try:
            base = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            lidar = tf.lookup_transform('map', 'lidar_link', rclpy.time.Time())
        except Exception:
            continue
        last = now
        pts = np.zeros((0, 2))
        s = st['scan']
        if s is not None:
            r = np.array(s.ranges)
            a = s.angle_min + np.arange(len(r)) * s.angle_increment
            ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
            ly = yaw(lidar.transform.rotation)
            lx, lyy = lidar.transform.translation.x, lidar.transform.translation.y
            pts = np.stack([lx + r[ok] * np.cos(a[ok] + ly), lyy + r[ok] * np.sin(a[ok] + ly)], 1)
        i = st['map'].info
        np.savez_compressed(
            f'{out}/{k:05d}.npz', t=now,
            grid=np.array(st['map'].data, dtype=np.int8).reshape(i.height, i.width),
            res=i.resolution, origin=(i.origin.position.x, i.origin.position.y),
            pose=(base.transform.translation.x, base.transform.translation.y,
                  yaw(base.transform.rotation)),
            plan=st['plan'], scan=pts.astype(np.float32))
        k += 1


if __name__ == '__main__':
    main()
