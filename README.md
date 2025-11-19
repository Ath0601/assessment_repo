# XArm5 Pick-and-Place Simulation  
### Gazebo Harmonic • MoveIt2 • Octomap • RGB-D Camera • ROS 2 Humble

This repository contains a full pick-and-place simulation of the **XArm5 robot**, including:

- Parallel gripper  
- RGB + Depth camera  
- Real-time Gazebo Harmonic simulation  
- Model pose bridges  
- MoveIt2 motion planning  
- Octomap mapping using RGB-D point cloud  
- Rviz2 visualization  

The gz_ros2_control package was built from source for use along with ROS2 Humble and Gazebo Harmonic.

---

## Workspace Structure
```
assessment_ws/
├── src/
│ ├── pickplace/ # Gazebo worlds, launch files, robot + sensors
│ └── moveit_config/ # MoveIt2 configuration package
```

## Installation

Install required ROS 2 Humble dependencies:

```bash
sudo apt update
sudo apt install \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-gz-ros2-control \
    ros-humble-moveit \
    ros-humble-tf2-tools
```

## Create a workspace
```
mkdir -p assessment_ws/
cd assessment_ws/
```

## Build the workspace
```
cd ~/assessment_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch Simulation
```
ros2 launch pickplace xarm.launch.py
```

This will start:
- **Gazebo simulation**  
- **ROS 2 control controllers**  
- **RGB + Depth + PointCloud bridges**  
- **MoveIt2 `move_group`**  
- **Octomap pipeline**  
- **RViz2 visualization**

## Video Demonstration
[**Video Link**](https://raw.githubusercontent.com/Ath0601/assessment_repo/main/src/Screencast%20from%2011-19-2025%2005%3A57%3A26%20PM.webm)

## Current Status
  The simulation launches with Gazebo Harmonic, RViz2 and MoveIt2. RGB and RGB-D cameras have been implemented and are 
running properly. The boxes spawn in the Gazebo environment but do not spawn in the RViz currently. The Octomap still 
needs to be implemented and still has some errors.
