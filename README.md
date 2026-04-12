# AMR Waiter Robot

A ROS 2-based autonomous/mobile waiter robot project with Arduino motor control, LiDAR filtering, dead-reckoning odometry, keyboard teleoperation, and robot visualization using URDF + RViz.

## Features

- ROS 2 Python package: `waiter_b`
- Arduino bridge for sending robot velocity commands and receiving IMU data
- LiDAR scan filtering node
- Dead-reckoning odometry node
- Keyboard teleoperation
- Robot visualization using URDF/Xacro and RViz
- Launch files for bringup, bridge, teleop, and SLAM workflow setup

## Project Structure

```bash
amr_waiter_robot-master/
├── arduino_codes/
│   └── sketch.ino
├── config/
│   ├── ekf.yaml
│   ├── lidar_filter.yaml
│   ├── slam.yaml
│   └── view.rviz
├── launch/
│   ├── bridge.launch.py
│   ├── bringup.launch.py
│   ├── ekf.launch.py
│   ├── remote_teleop.launch.py
│   └── slam.launch.py
├── urdf/
│   └── waiter_bot.urdf.xacro
├── waiter_b/
│   ├── __init__.py
│   ├── arduino_bridge.py
│   ├── dead_reckon.py
│   ├── lidar_filter.py
│   └── teleop.py
├── package.xml
├── setup.py
└── setup.cfg

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
