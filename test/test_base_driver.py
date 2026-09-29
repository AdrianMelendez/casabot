"""base_driver against an emulated ESP32 on a pseudo-terminal.

Needs ROS 2 and the built workspace, but no hardware:

    source install/setup.bash && python3 src/casabot/test/test_base_driver.py

The fake ESP32 speaks the serial protocol in docs/hardware.md: it takes
`v <left> <right>` wheel speeds, turns them into encoder ticks, and streams
`o` (ticks) and `i` (IMU) lines at 50 Hz, like the firmware. Checks that
cmd_vel reaches the wheels, odometry integrates correctly, and the wheels stop
when commands stop.
"""

import math
import os
import select
import subprocess
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TICKS_PER_REV, RADIUS, SEPARATION = 330.0, 0.0325, 0.20   # base_driver defaults


class FakeESP32(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)          # no echo: our own output must not come back as input
        self.port = os.ttyname(self.slave)
        self.left = self.right = 0.0    # rad/s commanded
        self.ticks_l = self.ticks_r = 0.0
        self.first_stop = None          # monotonic time wheels were first set to 0 after moving
        self.running = True

    def run(self):
        buf, last, last_report = b'', time.monotonic(), 0.0
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.005)
            if ready:
                buf += os.read(self.master, 1024)
                *lines, buf = buf.split(b'\n')
                for line in lines:
                    parts = line.decode(errors='ignore').split()
                    if len(parts) == 3 and parts[0] == 'v':
                        left, right = float(parts[1]), float(parts[2])
                        if left == right == 0.0 and (self.left or self.right):
                            self.first_stop = time.monotonic()
                        self.left, self.right = left, right
            now = time.monotonic()
            dt, last = now - last, now
            self.ticks_l += self.left * dt * TICKS_PER_REV / (2 * math.pi)
            self.ticks_r += self.right * dt * TICKS_PER_REV / (2 * math.pi)
            if now - last_report >= 0.02:
                last_report = now
                gz = (self.right - self.left) * RADIUS / SEPARATION
                os.write(self.master, f'o {int(self.ticks_l)} {int(self.ticks_r)}\n'.encode())
                os.write(self.master, f'i 0.0 0.0 9.81 0.0 0.0 {gz:.5f}\n'.encode())


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    esp = FakeESP32()
    esp.start()
    driver = subprocess.Popen(['ros2', 'run', 'casabot', 'base_driver', '--ros-args',
                               '-p', f'port:={esp.port}'],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('base_driver_test')
    odom, imu = {}, {}
    node.create_subscription(Odometry, 'odom', lambda m: odom.update(
        x=m.pose.pose.position.x, y=m.pose.pose.position.y, yaw=yaw(m.pose.pose.orientation),
        vx=m.twist.twist.linear.x, wz=m.twist.twist.angular.z, frame=m.header.frame_id,
        child=m.child_frame_id), 10)
    node.create_subscription(Imu, 'imu/data_raw', lambda m: imu.update(gz=m.angular_velocity.z), 10)
    cmd = node.create_publisher(Twist, 'cmd_vel', 10)

    def spin(seconds, vx=None, wz=None):
        t = time.monotonic()
        while time.monotonic() - t < seconds:
            if vx is not None:
                m = Twist()
                m.linear.x, m.angular.z = vx, wz
                cmd.publish(m)
            rclpy.spin_once(node, timeout_sec=0.1)

    try:
        t = time.monotonic()
        while 'x' not in odom and time.monotonic() - t < 20:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert 'x' in odom, 'no /odom from base_driver'
        assert odom['frame'] == 'odom' and odom['child'] == 'base_footprint'
        print('ok  base_driver publishes odom -> base_footprint')

        spin(3.0, vx=0.1, wz=0.0)
        expected = 0.1 / RADIUS
        assert abs(esp.left - expected) < 0.01 and abs(esp.right - expected) < 0.01, (esp.left, esp.right)
        assert abs(odom['vx'] - 0.1) < 0.01, odom
        assert 0.27 < odom['x'] < 0.33 and abs(odom['y']) < 0.01, odom
        print('ok  0.1 m/s for 3 s: wheels at %.3f rad/s, odom x = %.3f m' % (esp.left, odom['x']))

        quiet = time.monotonic()
        spin(1.0)                                   # stop publishing cmd_vel
        assert esp.left == 0.0 and esp.right == 0.0, (esp.left, esp.right)
        delay = esp.first_stop - quiet
        assert delay < 0.5 + 0.2, delay             # cmd_timeout, plus one 0.1 s check and slack
        print('ok  wheels stopped %.2f s after cmd_vel went quiet (cmd_timeout 0.5 s)' % delay)

        x0, yaw0 = odom['x'], odom['yaw']
        spin(3.0, vx=0.0, wz=0.5)
        turned = math.atan2(math.sin(odom['yaw'] - yaw0), math.cos(odom['yaw'] - yaw0))
        assert abs(esp.left + esp.right) < 1e-6, (esp.left, esp.right)
        assert 1.35 < turned < 1.65 and abs(odom['x'] - x0) < 0.01, (turned, odom)
        assert abs(imu['gz'] - 0.5) < 0.02, imu
        print('ok  0.5 rad/s for 3 s: turned %.2f rad on the spot, gyro %.3f rad/s' % (turned, imu['gz']))

        # Knock the USB cable out: the driver must exit with a clear error.
        esp.running = False
        esp.join()
        os.close(esp.master)
        os.close(esp.slave)
        code = driver.wait(timeout=10)
        log = driver.stdout.read()
        assert code != 0 and 'lost the base controller' in log and 'Traceback' not in log, log
        print('ok  unplugged: base_driver exits with "lost the base controller", no traceback')
        print('all base_driver checks passed')
    finally:
        if driver.poll() is None:
            driver.terminate()
            driver.wait(timeout=10)
        esp.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
