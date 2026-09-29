"""Serial bridge between ROS 2 and the ESP32 that owns the motors and the IMU.

Protocol (newline-delimited ASCII, 115200 baud), see docs/protocol.md:

    ROS  -> ESP32 :  v <left_rad_s> <right_rad_s>
    ESP32 -> ROS  :  o <left_ticks> <right_ticks>
    ESP32 -> ROS  :  i <ax> <ay> <az> <gx> <gy> <gz>

Odometry is published without a TF: robot_localization owns odom -> base_footprint.
"""

import math
from collections import deque

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from casabot.kinematics import integrate, ticks_to_rad, twist_to_wheels, wheels_to_twist, yaw_to_quaternion


class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        # Measure these on the real robot; the defaults are for the BOM chassis.
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.20)
        # One edge per encoder pulse, as the firmware counts: PPR x gear ratio.
        # 330 for the BOM's JGB37-520 (11 PPR, 30:1). Must match the firmware.
        self.declare_parameter('ticks_per_rev', 330.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.serial = serial.Serial(
            self.get_parameter('port').value,
            self.get_parameter('baud').value,
            timeout=0.05,
        )

        self.x = self.y = self.theta = 0.0
        self.last_ticks = None
        # (time, left, right) for the last VELOCITY_WINDOW seconds of ticks.
        self.tick_history = deque()
        self.last_cmd_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.02, self.poll_serial)
        self.create_timer(0.1, self.check_cmd_timeout)

        self.get_logger().info(f'base_driver up on {self.serial.port}')

    # -- outgoing ---------------------------------------------------------

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        left, right = twist_to_wheels(
            msg.linear.x, msg.angular.z, self.wheel_separation, self.wheel_radius
        )
        self.send_wheel_speeds(left, right)

    def send_wheel_speeds(self, left, right):
        try:
            self.serial.write(f'v {left:.4f} {right:.4f}\n'.encode())
        except OSError as e:            # pyserial's SerialException is an OSError too
            self.serial_lost(e)

    def serial_lost(self, error):
        """The ESP32 went away (USB knocked out, board reset). Exit with a clear
        message rather than a traceback from inside a timer; the firmware's own
        0.5 s command timeout has already stopped the motors."""
        self.get_logger().fatal(f'lost the base controller on {self.serial.port}: {error}')
        raise SystemExit(1)

    def check_cmd_timeout(self):
        """Stop if the planner goes quiet. A robot that keeps its last command
        after the controller dies drives into a wall."""
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.send_wheel_speeds(0.0, 0.0)

    # -- incoming ---------------------------------------------------------

    def poll_serial(self):
        try:
            self.read_lines()
        except OSError as e:
            self.serial_lost(e)

    def read_lines(self):
        while self.serial.in_waiting:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            try:
                self.handle_line(line)
            except (ValueError, IndexError):
                self.get_logger().warn(f'bad serial line: {line!r}')

    def handle_line(self, line):
        kind, *fields = line.split()
        if kind == 'o':
            self.handle_odom(int(fields[0]), int(fields[1]))
        elif kind == 'i':
            self.handle_imu([float(f) for f in fields[:6]])

    # Seconds of tick history the velocity is taken over. Serial lines arrive in
    # bursts (USB batches them), so message-to-message velocity swings wildly:
    # under load it read 0.06 m/s for a steady 0.1. At 330 ticks per wheel turn
    # a 20 ms message also holds only ~3 ticks, a third of a reading per tick.
    VELOCITY_WINDOW = 0.2

    def handle_odom(self, left_ticks, right_ticks):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.last_ticks is None:
            self.last_ticks = (left_ticks, right_ticks)
            self.tick_history.append((t, left_ticks, right_ticks))
            return

        # Position: integrate every tick, whatever the timing.
        d_left_rad = ticks_to_rad(left_ticks - self.last_ticks[0], self.ticks_per_rev)
        d_right_rad = ticks_to_rad(right_ticks - self.last_ticks[1], self.ticks_per_rev)
        self.last_ticks = (left_ticks, right_ticks)
        self.x, self.y, self.theta = integrate(
            self.x, self.y, self.theta,
            d_left_rad * self.wheel_radius,
            d_right_rad * self.wheel_radius,
            self.wheel_separation,
        )

        # Velocity: ticks over the last VELOCITY_WINDOW seconds, not the last message.
        self.tick_history.append((t, left_ticks, right_ticks))
        while len(self.tick_history) > 2 and self.tick_history[1][0] <= t - self.VELOCITY_WINDOW:
            self.tick_history.popleft()
        t0, l0, r0 = self.tick_history[0]
        span = t - t0
        if span <= 0.0:
            return
        vx, wz = wheels_to_twist(
            ticks_to_rad(left_ticks - l0, self.ticks_per_rev) / span,
            ticks_to_rad(right_ticks - r0, self.ticks_per_rev) / span,
            self.wheel_separation, self.wheel_radius,
        )

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = yaw_to_quaternion(self.theta)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        # Wheel odometry is decent in x/yaw and meaningless sideways.
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

    def handle_imu(self, values):
        ax, ay, az, gx, gy, gz = values
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        msg.orientation_covariance[0] = -1.0  # no absolute orientation from a 6-axis IMU
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.002
            msg.linear_acceleration_covariance[i] = 0.04
        self.imu_pub.publish(msg)

    def destroy_node(self):
        try:
            self.serial.write(b'v 0.0000 0.0000\n')
            self.serial.close()
        except OSError:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
