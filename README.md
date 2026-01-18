## agribot src.

This repository is a ROS 2 workspace used to develop and simulate mobile robots, with the current focus on Agribot, a differential-drive autonomous farming rover.


This is a repo where i will be sharing the build of my agribot . I am learning and applying it to my 6 wheel differential drive agribot. This is the way i am sharing my learnings into this repo. It's completely for the ground based UAV. 


agribot_description is where we defined the agribot urdf, meshes and launch files.

Repository Structure
agribot/
├── src/
│   ├── agribot_description/    # URDF/Xacro, meshes, RViz config
│   ├── agribot_controller/     # ros2_control configs & controllers
│   ├── agribot_bringup/        # Top-level launch files (sim/real)
│
├── docs/                       # Architecture & usage documentation
├── tools/                      # Helper scripts (future use)
│
├── build/                      # colcon build output (gitignored)
├── install/                    # colcon install output (gitignored)
├── log/                        # colcon logs (gitignored)
│
├── .gitignore
└── README.md

## Packages Overview
`agribot_description`

Contains the physical and visual description of the robot.

Includes:

* URDF/Xacro model

* Meshes

* RViz configuration

* Gazebo tags (where required)

Does not contain:

* Controllers

* Navigation

* Localization parameters

`agribot_controller`

Contains all ros2_control–related configuration.

Includes:

* ros2_control hardware and controller YAML

* Controller definitions (currently velocity-based for diff drive)

* Controller spawner logic (if applicable)

Purpose:

* Convert /cmd_vel into wheel-level commands

* Publish wheel-based odometry


`agribot_bringup`

The only package responsible for full system launches.

Includes:

* Gazebo simulation launch

* Robot state publisher

* ros2_control startup

* Bridges and teleop wiring (as needed)

Rule:

* If it launches the whole robot, it belongs here.

Requirements

1. Ubuntu 22.04

2. ROS 2 (Humble recommended)

3. Gazebo (via ros_gz_sim)

4. colcon

5. rosdep