# Magni Robot
## Installation
1. Clone this repository into your workspace
2. Clone [velodyne_description](http://wiki.ros.org/velodyne_description) if you want to use 3D Velodyne laser scanners
3. Clone [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) if you want to use realsense D435
4. catkin_make

## How to run
1. Run the following command to start up RVIZ and Gazebo with the magni robot

```
roslaunch magni_gazebo followme_test.launch
```

2. Run the twist keyboard to move the magni robot using keystrokes

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/ubiquity_velocity_controller/cmd_vel
```

3. To run magni robot on autonomous navigation instead, run the magni navigation node

```
roslaunch magni_nav move_base.launch
```

4. Once the `move_base.launch` launch file is executed, configure RVIZ as follows:

    1. At the left-hand side window, ensure that Global Options > Fixed Frame is set to `map`
    2. From the file menu bar, navigate to Panels > Tool Properties and change `2D nav goal` topic to `/move_base_simple/goal`
    3. Set the initial position of the magni robot using 2D pose estimation
    4. Set the destination position of the magni robot using 2D nav goal

## Actively Using packages:
- magni_gazebo
- magni_teleop
- magni_description
- magni_teleop

<p align="center">
  <img src="./images/gazebo.png">
  Gazebo interface
</p>

<p align="center">
  <img src="./images/rviz.png">
  RVIZ interface
</p>

## Notes:
1. You can check the full original README [here](https://github.com/UbiquityRobotics/magni_robot).
