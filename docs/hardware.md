# Hardware

## Bill of materials

Prices are rough 2026 street prices, for orientation only.

| Part | Choice | Why | ~Cost |
|---|---|---|---|
| Compute | Raspberry Pi 5, 4 GB | Runs ROS 2 Jazzy natively; Nav2 + slam_toolbox fit comfortably in 4 GB | $60 |
| Storage | 32 GB A2 microSD (or NVMe HAT + SSD) | SD cards are the usual cause of mystery corruption; an SSD is worth it if you reflash often | $10–45 |
| Lidar | RPLIDAR A1M8 (or C1) | 12 m range, 360°, `rplidar_ros` supports it out of the box | $100 |
| IMU | MPU6050 breakout | Gyro kills the yaw drift wheel odometry has on carpet | $3 |
| Microcontroller | ESP32 DevKitC | Real-time PID and encoder counting the Pi should not be doing | $6 |
| Motors | 2 × 6 V–12 V gearmotor with quadrature encoder, ~30:1 | Encoders are non-negotiable — no encoders, no odometry, no map | $25 |
| Motor driver | TB6612FNG breakout | More efficient and cooler than an L298N, same wiring effort | $6 |
| Wheels | 2 × 65 mm | Sets `wheel_radius = 0.0325` | $6 |
| Caster | 1 × ball caster | Low friction so it does not fight turns | $3 |
| Battery | 3S Li-ion pack or 2 × 18650 + 5 V BEC | Pi 5 wants a solid 5 V/5 A; browning out mid-map is the classic failure | $25 |
| Chassis | Laser-cut acrylic or 3D-printed plate | Any flat plate works; keep the lidar unobstructed 360° | $15 |

Total: roughly **$260**.

Sanity check before buying: the lidar must see over the whole robot. If anything
sticks up beside it, you get a permanent blind wedge in every map.

## Wiring

ESP32 pins, as set at the top of `firmware/esp32_base/esp32_base.ino`:

| Signal | ESP32 pin |
|---|---|
| Left motor PWM / IN1 / IN2 | 25 / 26 / 27 |
| Right motor PWM / IN1 / IN2 | 33 / 32 / 14 |
| TB6612 STBY | 12 |
| Left encoder A / B | 34 / 35 |
| Right encoder A / B | 36 / 39 |
| MPU6050 SDA / SCL | 21 / 22 |

GPIO 34–39 are input-only on the ESP32, which is exactly what encoder channels
need, and it keeps the output-capable pins free.

Power: motors run off the battery through the TB6612, **not** off the ESP32's
5 V rail. Share a common ground between the battery, the TB6612, the ESP32 and
the Pi, or the encoder signals will be noise.

The lidar and the ESP32 both enumerate as USB serial devices, and which one
becomes `/dev/ttyUSB0` is a coin flip at boot. Pin them with a udev rule:

```bash
# /etc/udev/rules.d/99-casabot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="casabot_lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="casabot_base"
```

Check your own IDs with `lsusb`, then launch with
`lidar_port:=/dev/casabot_lidar base_port:=/dev/casabot_base`.

## Serial protocol

ASCII, newline-delimited, 115200 baud. Deliberately readable so you can debug it
with `screen /dev/ttyUSB1 115200` and no ROS running at all.

| Direction | Message | Meaning |
|---|---|---|
| ROS → ESP32 | `v <left_rad_s> <right_rad_s>` | Wheel speed setpoints |
| ESP32 → ROS | `o <left_ticks> <right_ticks>` | Cumulative encoder counts, 50 Hz |
| ESP32 → ROS | `i <ax> <ay> <az> <gx> <gy> <gz>` | m/s², rad/s in `imu_link`, 50 Hz |

Both sides stop the motors if they stop hearing `v` for 0.5 s.

## Calibration

The physical robot never matches the numbers in the URDF. Three values decide
whether your map comes out square, and all three are measured, not guessed:

**`ticks_per_rev`** — Lift the robot off the ground. Mark a wheel, spin it
exactly 10 turns by hand, read the tick delta on the serial port, divide by 10.

**`wheel_radius`** — Command a straight 2 m drive, measure what the robot
actually travelled, then scale:
`wheel_radius_new = wheel_radius_old × (measured / 2.0)`.

**`wheel_separation`** — Command exactly 10 full rotations in place, measure the
final heading error. If it under-rotates, the value is too large. Scale:
`separation_new = separation_old × (commanded / actual)`.

Set the corrected values in all three places that hold them — they are not
shared between the driver, the model and the simulator:

| File | What to change |
|---|---|
| `casabot/base_driver.py` | `wheel_radius`, `wheel_separation`, `ticks_per_rev` parameter defaults (or override them in `robot.launch.py`) |
| `urdf/casabot.urdf.xacro` | the `wheel_radius` and `wheel_separation` xacro properties |
| `urdf/gazebo.xacro` | the `DiffDrive` plugin block — simulation only |
| `firmware/esp32_base/esp32_base.ino` | `TICKS_PER_REV` |

A robot whose odometry is 3 % off will still build a map; it will just be 3 %
the wrong size, and loop closure will fight it in every corridor.
