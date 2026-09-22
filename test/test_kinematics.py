"""Odometry maths only. No ROS, no hardware: `python3 test/test_kinematics.py`.

If these pass and the robot still drifts, the problem is wheel_radius,
wheel_separation or ticks_per_rev - measure them, do not tune the code.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from casabot.kinematics import (  # noqa: E402
    integrate, quaternion_to_yaw, ticks_to_rad, twist_to_wheels, wheels_to_twist, yaw_to_quaternion,
)

SEP = 0.20
RADIUS = 0.0325


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_straight_line():
    left, right = twist_to_wheels(0.5, 0.0, SEP, RADIUS)
    assert close(left, right)
    assert close(left * RADIUS, 0.5)


def test_spin_in_place():
    left, right = twist_to_wheels(0.0, 1.0, SEP, RADIUS)
    assert close(left, -right)
    _, wz = wheels_to_twist(left, right, SEP, RADIUS)
    assert close(wz, 1.0)


def test_twist_round_trip():
    for v, w in [(0.3, 0.0), (0.0, -1.2), (0.26, 0.9), (-0.1, 0.4)]:
        vx, wz = wheels_to_twist(*twist_to_wheels(v, w, SEP, RADIUS), SEP, RADIUS)
        assert close(vx, v) and close(wz, w)


def test_integrate_straight():
    x, y, theta = integrate(0.0, 0.0, 0.0, 1.0, 1.0, SEP)
    assert close(x, 1.0) and close(y, 0.0) and close(theta, 0.0)


def test_integrate_pure_rotation():
    """Equal and opposite wheel travel turns on the spot, no translation."""
    arc = SEP / 2 * math.pi          # half a turn
    x, y, theta = integrate(0.0, 0.0, 0.0, -arc, arc, SEP)
    assert close(x, 0.0) and close(y, 0.0)
    assert close(abs(theta), math.pi, 1e-9)


def test_integrate_quarter_circle():
    """Arc solution, not the straight-line approximation: driving a quarter of a
    circle of radius 1 must land at (1, 1) facing +y, in a single step."""
    d_center = math.pi / 2
    d_theta = math.pi / 2
    delta = d_theta * SEP / 2
    x, y, theta = integrate(0.0, 0.0, 0.0, d_center - delta, d_center + delta, SEP)
    assert close(x, 1.0, 1e-9) and close(y, 1.0, 1e-9)
    assert close(theta, math.pi / 2, 1e-9)


def test_integrate_wraps_angle():
    _, _, theta = integrate(0.0, 0.0, 3.0, -0.5, 0.5, SEP)
    assert -math.pi <= theta <= math.pi


def test_ticks():
    assert close(ticks_to_rad(1320, 1320.0), 2 * math.pi)
    assert close(ticks_to_rad(-660, 1320.0), -math.pi)


def test_quaternion_round_trip():
    for yaw in [0.0, 1.0, -2.5, math.pi / 2]:
        assert close(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw)


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all kinematics checks passed')
