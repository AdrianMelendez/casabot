# Bring-up: from parts on the desk to a first map

Do these in order. Each step checks one thing, so when something is wrong you
find it where it is, not three layers up as a map that spins or a robot that
drives backwards.

> Written from the code and the emulated tests, not from a built robot. Expect
> to adjust signs and gains; the steps say where.

Wiring and part choices are in [hardware.md](hardware.md). Commands below use
`/dev/ttyUSB1` for the ESP32 and `/dev/ttyUSB0` for the lidar; after step 5
they have stable names.

## 1. Software on the Pi

Ubuntu Server 24.04 (64-bit) on the Pi 5, then ROS 2 Jazzy from
[docs.ros.org](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

```bash
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
mkdir -p ~/ws/src && cd ~/ws/src && git clone https://github.com/AdrianMelendez/casabot.git
cd ~/ws && rosdep install --from-paths src --ignore-src -y --skip-keys "ros_gz_sim ros_gz_bridge rviz2"
colcon build --symlink-install
echo 'source ~/ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc      # same as the dev container
sudo usermod -aG dialout $USER                   # serial ports; log out and back in
```

The skipped keys are the simulator and RViz; run RViz on your laptop.

## 2. ESP32 alone: telemetry

Motor power **off**. Flash `firmware/esp32_base/esp32_base.ino` (Arduino IDE,
ESP32 core 3.3.x), plug the ESP32 into the Pi, then:

```bash
python3 ~/ws/src/casabot/tools/base_check.py /dev/ttyUSB1 listen
```

Expect about 50 odometry and 50 IMU lines per second, ticks at 0, and
`az` near **+9.8** with the robot flat. If `az` is near −9.8 the MPU6050 is
mounted upside down: flip it. The EKF takes yaw rate from its `gz`, and upside
down that sign is reversed, which turns every corner the wrong way in the map.

No lines at all: wrong port (`ls /dev/ttyUSB*`), or the sketch is not running.

## 3. Encoders: direction and ticks per turn

Motor power still **off**. Mark each wheel. Turn the **left** wheel by hand,
forward (the way it turns when the robot drives forward), exactly 10 turns, and
run `listen` again:

- ticks should rise by about **3300** (330 per turn for the BOM motor: 11 pulses
  per motor turn × 30:1 gearbox, counting one edge per pulse).
- if they **fall**, swap that encoder's A and B wires (or flip the `? 1 : -1`
  in `onLeftA` in the firmware).
- the exact count ÷ 10 is your `ticks_per_rev`. If it is not 330, set it in the
  firmware (`TICKS_PER_REV`), `base_driver` (`ticks_per_rev`) and
  `tools/base_check.py`.

Then the right wheel, the same way (`onRightA`).

## 4. Motors: direction, speed, watchdog

Wheels **off the ground**, motor power on.

```bash
python3 ~/ws/src/casabot/tools/base_check.py /dev/ttyUSB1 spin 3 3      # both forward
```

Both wheels should turn forward at about 29 rpm, and the report should read
`ok` for both, measured speed within 15% of 3 rad/s.

| You see | Fix |
|---|---|
| a wheel turns **backwards** and its ticks fall | swap that motor's two wires at the TB6612 |
| a wheel turns forward but its ticks fall | step 3 was skipped: encoder A/B swapped |
| speed far from 3 rad/s, or the wheel surges | PID gains `KP`/`KI` in the firmware; halve `KI` first |
| nothing moves | `STBY` pin, motor supply, common ground |

Then `spin -3 3`: the robot should turn **left** (counter-clockwise from
above), which is positive yaw in ROS. When the script ends, the wheels must stop
within half a second: that is the firmware watchdog, and it is what stops the
robot if the Pi crashes.

## 5. Stable port names, lidar, full bring-up

Add the udev rule from [hardware.md](hardware.md#wiring) so the ports become
`/dev/casabot_lidar` and `/dev/casabot_base`, then:

```bash
ros2 launch casabot robot.launch.py lidar_port:=/dev/casabot_lidar base_port:=/dev/casabot_base
ros2 topic hz /scan          # RPLIDAR A1: roughly 5-10 Hz
ros2 topic hz /odom          # 50 Hz
```

On the laptop (same `ROS_DOMAIN_ID`), open RViz, fixed frame `base_footprint`,
add LaserScan on `/scan`. Put a box one metre **in front** of the robot: its
dots must appear one metre along the red (x) axis. If they appear behind or to
the side, the lidar is mounted rotated: turn it, or add the rotation to
`lidar_joint` in `urdf/casabot.urdf.xacro`, e.g. `rpy="0 0 3.14159"`.

## 6. Calibrate

Drive with the keyboard, slowly:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
```

Follow [Calibration](hardware.md#calibration): a measured straight line for
`wheel_radius`, ten turns on the spot for `wheel_separation`. Watch
`ros2 topic echo /odometry/filtered --field pose.pose` while you do.

## 7. First map, then let it explore

Map one room by hand first:

```bash
ros2 launch casabot mapping.launch.py rviz:=false      # on the Pi; RViz on the laptop
```

Walls should come out straight and square. If they do not, the troubleshooting
section of the README goes through the causes in order.

Then let it explore, and **stay with it** the first time:

```bash
ros2 launch casabot explore.launch.py rviz:=false
```

It avoids what the lidar sees, one slice 11 cm off the floor. Stairs going
down, glass, and anything thinner or higher than that slice are invisible to
it. To stop it: Ctrl-C the launch (the wheels stop within 0.5 s on their own),
or cut the motor power.
