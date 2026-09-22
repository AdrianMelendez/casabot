"""Pure differential-drive math. No ROS imports, so it is unit-testable anywhere."""

import math


def ticks_to_rad(ticks, ticks_per_rev):
    """Encoder ticks -> wheel rotation in radians."""
    return 2.0 * math.pi * ticks / ticks_per_rev


def twist_to_wheels(linear_x, angular_z, wheel_separation, wheel_radius):
    """Body twist (m/s, rad/s) -> (left, right) wheel speeds in rad/s."""
    half_track = angular_z * wheel_separation / 2.0
    return (
        (linear_x - half_track) / wheel_radius,
        (linear_x + half_track) / wheel_radius,
    )


def wheels_to_twist(left_rad_s, right_rad_s, wheel_separation, wheel_radius):
    """(left, right) wheel speeds in rad/s -> body twist (m/s, rad/s)."""
    v_left = left_rad_s * wheel_radius
    v_right = right_rad_s * wheel_radius
    return (
        (v_right + v_left) / 2.0,
        (v_right - v_left) / wheel_separation,
    )


def integrate(x, y, theta, d_left, d_right, wheel_separation):
    """Advance pose by the distance each wheel travelled (metres).

    Uses the exact arc solution rather than the straight-line approximation;
    both are the same amount of code and the arc one does not drift on turns.
    """
    d_center = (d_left + d_right) / 2.0
    d_theta = (d_right - d_left) / wheel_separation

    if abs(d_theta) < 1e-9:
        x += d_center * math.cos(theta)
        y += d_center * math.sin(theta)
    else:
        radius = d_center / d_theta
        cx = x - radius * math.sin(theta)
        cy = y + radius * math.cos(theta)
        theta_new = theta + d_theta
        x = cx + radius * math.sin(theta_new)
        y = cy - radius * math.cos(theta_new)
        theta = theta_new

    return x, y, math.atan2(math.sin(theta), math.cos(theta))


def yaw_to_quaternion(yaw):
    """Yaw in radians -> (x, y, z, w). Avoids a transforms3d dependency."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
