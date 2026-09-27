## Overview

This project implements a **rough-terrain path planner for autonomous ground robots** using the **A* search algorithm**. The planner generates collision-free paths while taking terrain characteristics into account, enabling navigation over uneven and challenging terrain.

The project is developed with **ROS 2 Humble** and uses **RViz2** for real-time visualization of the terrain, planning map, and generated path.

### Features

* **Rough-terrain path planning** for autonomous ground robots
* **A* path planning algorithm**
* Terrain-aware navigation
* **ROS 2 Humble** integration
* **RViz2 visualization** of the planning environment and generated paths
* Configurable planning parameters through YAML

on docker:

    docker pull osrf/ros:humble-desktop

to run with rviz window (tested on x11):

    docker run -it --rm  --net=host  -e DISPLAY=$DISPLAY  -e QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix:rw   osrf/ros:humble-desktop

install packages:

    sudo apt update
  	sudo apt install ros-humble-rviz-visual-tools
    sudo apt install libeigen3-dev

setup:

    cd /home
    mkdir terrain_navigation
    cd terrain_navigation/
    git clone https://github.com/phat-truongxuan/terrain_navigation.git
    mv terrain_navigation src
    colcon build

start rviz:

    source install/setup.bash 
    ros2 launch path_planner map.launch.py

run nav nove (new terminal, you might have to open new terminal and run docker exec -it [container id] bash, check running container id with docker -ps):

    cd /home/terrain_navigation/
    source install/setup.bash 
    ros2 run path_planner nav_main

to find path (new terminal, might run docker exec -it [container id] bash again), publish start end location:

    cd /home/terrain_navigation/
    source install/setup.bash 
    ros2 topic pub --once /all_in_one path_planner/msg/AllInfo "{start_id_x: 10, start_id_y: 1, goal_id_x: 50, goal_id_y: 50, omega: 10, beta: 1, gamma: 50, mu: 0.8, color: 7, follow_path: True}"

## Paper

**Rough Terrain Path Planning for Autonomous Ground Robot**
Xuan-Phat Truong and Seong Hyeon Hong
*AIAA SCITECH 2024 Forum, 2024*

[📄 Paper](https://arc.aiaa.org/doi/10.2514/6.2024-2764)

## Citation

If you find this work useful, please cite:

```bibtex
@inproceedings{truong2024rough,
  title={Rough Terrain Path Planning for Autonomous Ground Robot},
  author={Truong, Xuan-Phat and Hong, Seong Hyeon},
  booktitle={AIAA SCITECH 2024 Forum},
  pages={2764},
  year={2024},
  doi={10.2514/6.2024-2764}
}
```
