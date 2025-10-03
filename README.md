# Integrated Brain ROS Workspace

This is the workspace of our awesome project!


## Getting started
The documentation for managing the workspace is split into various READMEs, please read the correct file using the (relative) links below.

| File | Purpose |
| --- | --- |
| [Setup Scripts](/config/scripts/README.md) | Instructions and shell scripts to help quickly launch the workspace without dependence on Devcontainers. |
| [Udev Rules](/config/udev/README.md) | Configure `udev` rules in the host OS to easily recognise the robot if plugged in via USB. |
| [Containerization Source Code](https://github.com/eccentricOrange/int-brain-containers/) \[hosted externally\] | Source code for the Dockerfiles for to building the containers used by this project. |

### Run NAV2 on SBC
1. Launch system 
    ```bash
    ros2 launch int_brain_system system.launch.py rviz:=false stamp_twist:=true
    ```

2. In a new terminal, launch navigation
    ```bash
    ros2 launch nav2_bringup bringup_launch.py slam:=True params_file:=/home/ubuntu/int_brain_ws/src/int_brain_navigation/config/nav2_params.yaml
    ```

## Acknowledgements
The Devcontainer system of this project was inspired by the Wheelchair project at [RRC, IIIT Hyderabad](https://github.com/Smart-Wheelchair-RRC/). Please see their project [DockerForDevelopment](https://github.com/Smart-Wheelchair-RRC/DockerForDevelopment).
