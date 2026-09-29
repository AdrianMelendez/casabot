# Hardware

## Bill of materials

Single-unit US prices, checked September 2026. Rows marked *est.* had no
listing with a visible price when checked; treat them as ballpark.

Two things moved a lot since this list was first written. Memory shortages
driven by AI data centres pushed the Raspberry Pi 5 4 GB from its $60 launch
price to $110, and microSD cards and SSDs up with it. Meanwhile the newer RPLIDAR
C1 now costs less than the older A1.

| Part | Choice | Why | Price | Source |
|---|---|---|---|---|
| Compute | Raspberry Pi 5, 4 GB | Runs ROS 2 Jazzy natively; Nav2 + slam_toolbox fit in 4 GB. The 8 GB is $180 and not needed | $110 | [CanaKit](https://www.canakit.com/raspberry-pi-5-4gb.html) |
| Cooling | Raspberry Pi Active Cooler | Nav2 and SLAM keep the CPU busy for minutes at a time; without cooling the Pi 5 throttles | $10.95 | [PiShop](https://www.pishop.us/product/raspberry-pi-active-cooler/) |
| Storage | 32 GB A2 microSD | SD cards are the usual cause of mystery corruption; buy a known brand | ~$10–15 *est.* | [NAND prices](https://www.tomshardware.com/pc-components/storage/memory-cards-and-flash-drives-prices-rocket-124-percent-some-products-peak-at-261-percent-jump-increases-from-2025-driven-by-ai-chip-shortage-across-a-range-of-formats-and-capacities) |
| Lidar | RPLIDAR A1M8 | 12 m, 360°, works with the apt `rplidar_ros` driver that `robot.launch.py` uses | $99 | [Seeed](https://www.seeedstudio.com/RPLiDAR-A1M8-R6-360-Degree-Laser-Scanner-Kit-12M-Range-p-4785.html) |
| *or* Lidar | RPLIDAR C1 | Cheaper and a better sensor (DTOF), but needs Slamtec's [`sllidar_ros2`](https://github.com/Slamtec/sllidar_ros2) built from source at 460800 baud | $69 | [Seeed](https://www.seeedstudio.com/RPLiDAR-C1M1-R2-Portable-ToF-Laser-Scanner-Kit-12M-Range-p-5840.html) |
| IMU | MPU6050 (GY-521) breakout | Gyro kills the yaw drift wheel odometry has on carpet | ~$5 *est.* | |
| Microcontroller | ESP32-DevKitC-32E | Real-time PID and encoder counting the Pi should not be doing | $10 | [DigiKey](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810) |
| Motors | 2 × JGB37-520 12 V gearmotor with Hall encoder, ~200–330 RPM | Encoders are non-negotiable: no encoders, no odometry, no map | $17–19 each | [Oz Robotics](https://ozrobotics.com/shop/encoder-reduction-motor-jgb37-520b-12v-dc-deceleration-motor-12v-333rpm/) |
| *or* Motors | 2 × Pololu 34:1 25D LP 6 V with 48 CPR encoder | Documented encoder (1632.67 counts/rev), consistent unit to unit | $53.95 each | [Pololu](https://www.pololu.com/product/4824) |
| Motor driver | TB6612 breakout | More efficient and cooler than an L298N, same wiring effort. 1.2 A per channel, so check your motor's stall current | $6.95 | [Adafruit](https://www.adafruit.com/product/2448) |
| Wheels | 2 × 65 mm | Sets `wheel_radius = 0.0325` | ~$8 *est.* | |
| Caster | 1 × ball caster | Low friction so it does not fight turns | ~$4 *est.* | |
| Battery | 3S Li-ion pack, plus a balance charger if you do not own one | Runs the motors directly | ~$30 *est.* | |
| 5 V supply | 5 V / 5 A step-down regulator | The Pi 5 wants 5 A; the common 3 A UBECs are not enough, and browning out mid-map is the classic failure. Pololu D36V50F5 shown; generic 5 A bucks run ~$15 | $39.95 | [Pololu](https://www.pololu.com/product/4091) |
| Chassis | Laser-cut acrylic or 3D-printed plate | Any flat plate works; keep the lidar unobstructed 360° | ~$15 *est.* | |

Totals:

- **Budget build, about $330:** RPLIDAR C1, JGB37-520 motors, a generic 5 A buck.
- **Fewer surprises, about $460:** RPLIDAR A1M8 on the packaged driver, Pololu
  motors, Pololu regulator.

Neither total includes shipping, wire, connectors or screws.

Keep the centre of mass between the wheel axle and the caster: put the battery
toward the caster. With the weight over the axle the robot tips onto its nose
every time it brakes, the low-mounted lidar sees the floor, and the map fills
with walls that are not there. The simulated model had exactly this bug.

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
