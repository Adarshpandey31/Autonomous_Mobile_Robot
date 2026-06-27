# AMR Waiter Robot (`waiter_b`)

A **ROS 2** autonomous mobile robot (AMR) — a four-wheel **mecanum** "waiter bot"
that drives in indoor environments, senses obstacles with a 2D LiDAR, and is
visualized live in RViz. A computer running ROS 2 handles high-level control,
sensing, and odometry, while an **Arduino**-based controller drives the motors
and reads an IMU over a serial link.

The package ships everything needed to bring the robot up, teleoperate it, and
extend it toward full SLAM-based autonomous navigation.

<p align="center">
  <em>Middleware:</em> ROS 2 &nbsp;•&nbsp;
  <em>Base:</em> 4-wheel mecanum (Cytron drivers) &nbsp;•&nbsp;
  <em>Sensing:</em> RPLidar + BNO055 IMU &nbsp;•&nbsp;
  <em>Mapping:</em> slam_toolbox
</p>

---

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Nodes & Topics](#nodes--topics)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Build](#build)
  - [Flash the Arduino](#flash-the-arduino)
- [How to Run](#how-to-run)
- [Configuration](#configuration)
- [Theory](#theory)

---

## Features

- **ROS 2 Python package** (`waiter_b`) with a modular, node-per-task design.
- **Arduino bridge** — streams `/cmd_vel` velocity commands to the microcontroller
  over serial and publishes IMU data back as `/imu/data`.
- **LiDAR scan filter** — cleans raw scans (range + angle gating) into a usable
  `/scan`.
- **Dead-reckoning odometry** — calibrated odometry from `/cmd_vel` (no wheel
  encoders), publishing `/odom` and the `odom → base_link` TF.
- **Keyboard teleoperation** for manual driving and hardware bring-up testing.
- **URDF/Xacro robot model** with `robot_state_publisher` + RViz visualization.
- **Launch files** for full bring-up, the serial bridge, SLAM, EKF sensor
  fusion, and remote (web) teleop.
- **SLAM-ready** — `slam_toolbox` and `robot_localization` (EKF) configs included.

## System Architecture

```text
                 ┌──────────────────────────────────────────────┐
                 │                   ROS 2 (PC)                  │
   keyboard ──▶  │  teleop ──▶ /cmd_vel ──▶ arduino_bridge ──┐   │
                 │                                            │   │
   RPLidar ──▶ /scan_raw ──▶ lidar_filter ──▶ /scan          │   │
                 │                                            │   │
                 │  dead_reckon ──▶ /odom + TF (odom→base)    │   │
                 │  robot_state_publisher ──▶ RViz            │   │
                 └────────────────────────────────────────────┼──┘
                                                  serial (USB) │
                 ┌────────────────────────────────────────────▼──┐
                 │         Arduino (low-level controller)         │
                 │  mecanum kinematics ──▶ 4× Cytron motor driver │
                 │  BNO055 IMU ──▶ /imu/data (back over serial)   │
                 └────────────────────────────────────────────────┘
```

The split keeps **high-level decision-making on the PC** and **real-time
hardware control on the microcontroller**, connected by a single serial link.

## Hardware

| Component | Part |
| --- | --- |
| Compute | PC running ROS 2 |
| Microcontroller | Arduino (e.g. Mega) |
| Drive base | 4× mecanum wheels |
| Motor drivers | Cytron (`PWM_DIR` mode) |
| IMU | Adafruit **BNO055** (I²C `0x28`) |
| LiDAR | RPLidar (`rplidar_ros`) |

> Robot geometry (wheel radius, `Lx`, `Ly`) and serial baud rate are set in
> [`arduino_codes/sketch.ino`](arduino_codes/sketch.ino). Default baud is
> `115200`.

## Nodes & Topics

| Node | Executable | Subscribes | Publishes |
| --- | --- | --- | --- |
| Arduino bridge | `arduino_bridge` | `/cmd_vel` | `/imu/data` |
| LiDAR filter | `lidar_filter` | `/scan_raw` | `/scan` |
| Dead-reckoning | `dead_reckon` | `/cmd_vel` | `/odom` + `odom→base_link` TF |
| Teleop | `teleop` | — | `/cmd_vel` |

## Project Structure

```text
waiter_b/
├── arduino_codes/
│   └── sketch.ino                # Firmware: mecanum kinematics, motors, BNO055 IMU
├── config/
│   ├── ekf.yaml                  # robot_localization EKF parameters
│   ├── lidar_filter.yaml         # LiDAR range/angle filter parameters
│   ├── slam.yaml                 # slam_toolbox parameters
│   └── view.rviz                 # RViz layout
├── launch/
│   ├── bringup.launch.py         # Full stack: bridge, lidar, filter, RSP, RViz, odom
│   ├── bridge.launch.py          # Arduino bridge + RPLidar only
│   ├── slam.launch.py            # Bring-up wired for SLAM
│   ├── ekf.launch.py             # robot_localization EKF node
│   └── remote_teleop.launch.py   # rosbridge web teleop + Arduino bridge
├── urdf/
│   └── waiter_bot.urdf.xacro     # Robot model
├── waiter_b/
│   ├── arduino_bridge.py         # Serial bridge: /cmd_vel ⇄ Arduino, IMU publish
│   ├── dead_reckon.py            # Calibrated dead-reckoning odometry
│   ├── lidar_filter.py           # Raw-scan cleanup node
│   └── teleop.py                 # Keyboard teleoperation
├── package.xml
├── setup.py
└── setup.cfg
```

## Getting Started

### Prerequisites

- **ROS 2** (Humble or newer) on Ubuntu, with `colcon` and `rosdep`.
- A built ROS 2 workspace (e.g. `~/ros2_ws`).
- ROS 2 dependencies (install via `rosdep`, or manually):
  - `robot_state_publisher`, `xacro`, `rviz2`, `tf2_ros`, `tf_transformations`
  - `rplidar_ros` (LiDAR driver)
  - `slam_toolbox` (mapping), `robot_localization` (EKF)
  - `rosbridge_server` (only for remote/web teleop)
- Python: `pyserial` (`pip install pyserial`) for the Arduino serial link.

### Build

```bash
# Place this package inside your workspace's src/ directory
cd ~/ros2_ws/src
git clone https://github.com/Adarshpandey31/Autonomous_Mobile_Robot.git waiter_b

# Install dependencies and build
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waiter_b
source install/setup.bash
```

### Flash the Arduino

Open [`arduino_codes/sketch.ino`](arduino_codes/sketch.ino) in the Arduino IDE,
install the required libraries (`Adafruit_BNO055`, `Adafruit_Sensor`,
`CytronMotorDriver`), tune the robot-geometry constants to match your chassis,
and upload it to the board.

> Confirm the serial device names (`/dev/ttyUSB0` for the Arduino,
> `/dev/ttyUSB1` for the LiDAR by default) and grant access with
> `sudo usermod -aG dialout $USER` if needed.

## How to Run

> `source install/setup.bash` in every new terminal first.

**Full bring-up** (Arduino bridge + LiDAR + filter + robot model + RViz + odometry):

```bash
ros2 launch waiter_b bringup.launch.py
# Override serial ports if needed:
ros2 launch waiter_b bringup.launch.py bridge_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

**Drive it** (in a second terminal):

```bash
ros2 run waiter_b teleop
```

**Hardware bridge only** (Arduino + LiDAR, no visualization):

```bash
ros2 launch waiter_b bridge.launch.py
```

**SLAM mapping** (run alongside `slam_toolbox`):

```bash
ros2 launch waiter_b slam.launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=config/slam.yaml
```

**EKF sensor fusion** (fuse odometry + IMU):

```bash
ros2 launch waiter_b ekf.launch.py
```

**Remote / web teleop** (via `rosbridge`):

```bash
ros2 launch waiter_b remote_teleop.launch.py
```

## Configuration

| File | What it controls |
| --- | --- |
| [`config/lidar_filter.yaml`](config/lidar_filter.yaml) | LiDAR range thresholds and front-facing angle window |
| [`config/slam.yaml`](config/slam.yaml) | `slam_toolbox` solver, frames, and map parameters |
| [`config/ekf.yaml`](config/ekf.yaml) | `robot_localization` EKF (2D mode, 50 Hz, sensor inputs) |
| [`config/view.rviz`](config/view.rviz) | RViz displays and layout |

Key launch arguments: `bridge_port`, `bridge_baud`, `lidar_port`, `lidar_baud`.
The dead-reckoning node exposes `linear_scale` / `angular_scale` calibration
gains (set in `bringup.launch.py`).

## Theory

This project is based on the working principles of a mobile robot built using ROS 2, an Arduino-based low-level controller, LiDAR sensing, IMU feedback, and odometry estimation. The purpose of the system is to control and monitor a waiter robot that can move in an indoor environment, sense obstacles, and be visualized in ROS tools like RViz.

### 1. ROS 2 in Robotics

ROS 2 (Robot Operating System 2) is a middleware framework used in robotics for communication between different software modules. Instead of writing one large program for the entire robot, ROS 2 allows the system to be divided into small nodes, where each node performs a specific task such as motor control, LiDAR processing, odometry estimation, or visualization.

In this project, ROS 2 is used to:
- send motion commands
- receive and publish sensor data
- launch multiple robot modules together
- visualize the robot and its data

This modular design makes the robot easier to debug, maintain, and extend.

### 2. Mobile Robot Motion Control

A mobile robot moves by converting high-level motion commands into wheel-level actions. In ROS 2, robot motion is commonly controlled using the `/cmd_vel` topic. This topic usually carries two main velocity components:
- linear velocity: forward or backward motion
- angular velocity: rotation of the robot

When the user gives commands through the keyboard teleoperation node, those commands are published as velocity messages. The Arduino bridge reads these commands and sends them over serial communication to the Arduino. The Arduino then drives the motors accordingly.

This creates a complete motion pipeline:

Keyboard Input → ROS 2 `/cmd_vel` → Arduino Bridge → Arduino → Motors

### 3. Serial Communication Between ROS 2 and Arduino

The robot uses serial communication as the link between the high-level ROS 2 software and the low-level Arduino controller. Serial communication is important because ROS 2 runs on the computer, while the Arduino directly controls hardware such as motors and sensors.

In this project:
- ROS 2 sends velocity commands to the Arduino
- Arduino receives these commands and drives the motors
- Arduino also reads sensor values such as IMU data
- sensor data is sent back to ROS 2 through the same serial connection

This architecture is useful because it separates:
- high-level decision making on the computer
- low-level real-time hardware control on the microcontroller

### 4. IMU and Orientation Feedback

An IMU (Inertial Measurement Unit) is a sensor used to measure motion-related values such as orientation, angular velocity, and acceleration. This project uses the BNO055 IMU.

The IMU helps the robot estimate how it is rotating or moving. This is useful because robot movement in the real world is often imperfect. Wheels may slip, surfaces may not be even, and commands may not produce exact motion. IMU feedback helps improve understanding of robot behavior.

The Arduino reads IMU values and sends them to ROS 2, where they can be used for monitoring, localization, or future sensor fusion.

### 5. LiDAR and Obstacle Sensing

LiDAR stands for Light Detection and Ranging. It measures the distance to surrounding objects by sending out laser beams and detecting their reflections. LiDAR is widely used in mobile robotics because it provides accurate distance measurements around the robot.

In this project, the LiDAR data is used to sense the surrounding environment. However, raw LiDAR scans may include:
- invalid values
- noise
- unwanted ranges
- unnecessary angle regions

That is why a LiDAR filter node is used. The filter processes the raw scan and republishes a cleaner version of the data. Filtered scan data is more reliable for visualization, mapping, or future navigation tasks.

### 6. Odometry and Dead Reckoning

Odometry is the process of estimating the position and orientation of a robot over time. Dead reckoning is one method of odometry in which the robot estimates its movement using its commanded motion or wheel movement.

In this project, the dead reckoning node estimates robot motion from velocity commands and publishes odometry on the `/odom` topic.

This helps in:
- tracking robot movement over time
- visualizing robot pose in ROS tools
- providing position estimates for higher-level robotics applications

Dead reckoning is simple and useful, but it is not perfectly accurate. Small errors keep accumulating over time due to:
- wheel slip
- uneven surfaces
- motor inaccuracies
- unmodeled dynamics

Because of this, dead reckoning is often improved in advanced systems using IMU data, encoders, or EKF-based sensor fusion.

### 7. URDF and Robot Visualization

URDF stands for Unified Robot Description Format. It is used in ROS to describe the physical structure of a robot, including:
- links
- joints
- dimensions
- sensor and frame relationships

In this project, the robot model is defined using URDF/Xacro. This allows ROS tools such as `robot_state_publisher` and RViz to display the robot properly.

Visualization is important because it helps the developer:
- verify the robot model
- understand frame relationships
- inspect robot pose and sensor data
- debug integration issues

### 8. Teleoperation

Teleoperation means manually controlling the robot from a keyboard or another remote interface. In this project, the teleop node allows the user to move the robot using simple keyboard commands.

Teleoperation is useful for:
- testing the robot hardware
- checking motor response
- verifying serial communication
- validating odometry and sensor outputs
- moving the robot safely during development

Before implementing autonomy, teleoperation is usually the first and most important step in testing a mobile robot.

### 9. Launch Files in ROS 2

ROS 2 launch files are used to start multiple nodes together in a structured way. This is important in robotics because one working system usually depends on several nodes running at the same time.

For example, in this project, a launch file can start:
- the Arduino bridge
- the LiDAR node
- the LiDAR filter
- the robot state publisher
- RViz
- the dead reckoning node

This makes robot bringup easier, faster, and less error-prone.

### 10. SLAM and Future Navigation

The project also contains SLAM-related configuration files. SLAM stands for Simultaneous Localization and Mapping. It is a robotics technique in which the robot builds a map of the environment while also estimating its own position inside that map.

LiDAR is commonly used in SLAM because it provides accurate surrounding distance data. In a more advanced version of this project, the robot can use:
- LiDAR for sensing walls and objects
- odometry for motion estimation
- IMU for orientation stability
- SLAM for map building and localization

This would allow the waiter robot to move more autonomously in indoor environments.

### 11. Why This Architecture is Useful

The overall architecture of this project is useful because it separates the robot system into clear layers:

- **hardware layer**: motors, IMU, LiDAR, Arduino
- **communication layer**: serial communication and ROS topics
- **processing layer**: LiDAR filtering, odometry estimation
- **application layer**: teleoperation, visualization, bringup, and future navigation

This modular structure is widely used in robotics because it improves:
- scalability
- maintainability
- debugging
- future upgrades

### 12. Summary of Working Principle

The complete working principle of the project can be understood as follows:

1. The user sends motion commands using keyboard teleoperation.
2. ROS 2 publishes these commands on `/cmd_vel`.
3. The Arduino bridge reads the commands and sends them over serial to the Arduino.
4. The Arduino drives the motors and reads IMU data.
5. IMU data is sent back to ROS 2 through serial communication.
6. LiDAR provides scan data of the surroundings.
7. The LiDAR filter cleans the raw scan data.
8. The dead reckoning node estimates odometry.
9. The robot model and sensor information are visualized in RViz.
10. The system can later be extended for SLAM and autonomous navigation.

This makes the project a complete foundation for a mobile service robot or waiter robot.
