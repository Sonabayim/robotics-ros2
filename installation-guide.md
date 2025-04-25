
## Table of Contents

- [gazebo-ros-bridge](#gazebo-ros-bridge)
- [clone-and-build-turtlebot3-repo](#clone-and-build-turtlebot3-repo)
- [source-and-set-model](#source-and-set-model)
- [launch-world](#launch-world)
- [workspace-structure](#workspace-structure)
- [launch-options](#launch-options)
- [keyboard-teleop](#keyboard-teleop)
- [source-in-new-terminal](#source-in-new-terminal)
- [verify-sim-packages](#verify-sim-packages)
- [install-rviz](#install-rviz)
- [install-extra-packages](#install-extra-packages)
- [verify-cmd_vel](#verify-cmd_vel)
- [teleop-verification](#teleop-verification)
- [urdf-plugin-check](#urdf-plugin-check)
- [alternative-teleop](#alternative-teleop)
- [reporting-issues](#reporting-issues)



---

## Requirements 

This installation guide is suitable for ubuntu 22.04 and ros2 humble. Before starting installation make sure you have installed [ros2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). 


## gazebo-ros-bridge

the official TurtleBot3 Simulation packages still target Gazebo Classic, so we will install ros gazebo bridge packages. Alternatively, you could use ignition fortress, but it was hectic for me, so I used gazebo classic 11.

```bash
sudo apt update 
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-gazebo-ros-pkgs
```

---

## clone-and-build-turtlebot3-repo

In case of you have limited RAM, this is a **very common issue**, especially on laptops or virtual machines with limited RAM (e.g., 4–8 GB). The C++ compiler `cc1plus` is being **killed by the system (OOM - Out Of Memory)** during the build of a relatively heavy file:  
`traffic_light_plugin.cpp`. Use second colcon build command:

```bash
mkdir -p ~/turtlebot3_ws/src && cd $_

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

sudo apt install python3-colcon-common-extensions

cd ~/turtlebot3_ws
colcon build --symlink-install

# For limited RAM:
colcon build --symlink-install --executor sequential
```

---

## source-and-set-model

```bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## launch-world

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## workspace-structure

After building, the workspace should contain:

```
turtlebot3_ws/
├── src/
├── build/
├── install/
└── log/
```

---

## launch-options

### Full World Launch

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Empty World Launch

```bash
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
```

### Spawn Robot into Existing World

```bash
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py
```

---

## keyboard-teleop

In a new terminal:

```bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## source-in-new-terminal

Each time you open a new terminal you need to source environment for ros2 and turtlebot3. I would recommend to try these commands several time before adding the to `bashrc` file, as it helped to understand why I need those commands in the first place. 

```bash

source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
```


### how to automate sourcing


```bash

nano ~/.bashrc

```

you can use other text editors to edit this file, either way you will see several lines of codes and you need to go at the end of the file to add following commands that you use sourcing

```bash

# === ROS 2 Humble environment setup ===
source /opt/ros/humble/setup.bash

# === TurtleBot3 workspace setup ===
source ~/turtlebot3_ws/install/setup.bash

# === Specify TurtleBot3 model ===
export TURTLEBOT3_MODEL=waffle

```
commenting style is similar there, but it is up to you how you would to comment or leave it be. afterwards just use `Ctrl+O` and Enter, then `Ctrl+X` to exit the text editor from terminal. Afterwards restart the terminal and now you need only one line command 
`source ~/.bashrc
`
Shortly what each command lines mean. First line adds ros2 packages to the systems path and configures ros2 related environment variables. If you try to run ros2 commands in fresh terminal it will return error of not recognising the command of ros2. so in order to launch nodes and interact with ros2 setting path is important 
Second line is basically same thing, but this time we are setting path of turtlebot3. In order for ros2 to acknowledge turtlbot robot, it is important to setup workspace. For any robot you work  with you would need to setup custom workspace where ros2 can find and use custom build packages. Lastly, we specify the type of turtlebot we will work with. There are 3 types, and our work will involve `waffle` version of turtlebot. 

[What is bashrc?](bashrc.md)

---

## verify-sim-packages

Run:

```bash
ros2 pkg list | grep turtlebot3
```

Expected output should include:

- `turtlebot3`
    
- `turtlebot3_msgs`
    
- `turtlebot3_description`
    
- `turtlebot3_gazebo`
    

If `turtlebot3_gazebo` is missing, clone it manually:

```bash
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

---

## install-rviz

install the rviz and setup with gazebo turtlebot3

``` bash

sudo apt update
sudo apt install ros-humble-rviz2

# Gazebo–ROS bridge & tools
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image

# Navigation 2 and TurtleBot3 messages / description
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3-msgs ros-humble-turtlebot3-description

```

## environment-setup

```bash

source /opt/ros/humble/setup.bash
source ~/tb3_ign_ws/install/setup.bash

# Tell TurtleBot3 which model to use
export TURTLEBOT3_MODEL=waffle        # or burger, waffle_pi

# Make sure Gazebo can find the robot meshes/textures
export GZ_SIM_RESOURCE_PATH=$HOME/tb3_ign_ws/src/nav2_minimal_turtlebot_simulation/nav2_minimal_tb3_sim/models:$GZ_SIM_RESOURCE_PATH

```

## install-extra-packages

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

---

## verify-cmd_vel

Run:

```bash
ros2 topic info /cmd_vel
```

Expected output:

```
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscriber count: 1
```

---

## teleop-verification

### Terminal A - Launch Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal B - Start Teleop

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Expected output:

```
Reading from keyboard
---------------------------
Use 'WASD' keys to move the robot,  
press 'q' to quit.
```

### Terminal C - Monitor Commands

```bash
ros2 topic echo /cmd_vel
```

Pressing W/A/S/D in Terminal B should show non-zero velocity messages.

---

## urdf-plugin-check

If no movement occurs, inspect:

```
turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf.xacro
```

Verify it contains the following plugin:

```xml
<plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/</namespace>
    <remapping>cmd_vel:=/cmd_vel</remapping>
  </ros>
  <leftJoint>left_wheel_joint</leftJoint>
  <rightJoint>right_wheel_joint</rightJoint>
  <wheelSeparation>0.287</wheelSeparation>
  <wheelDiameter>0.066</wheelDiameter>
  <updateRate>1000</updateRate>
</plugin>
```

If not present, add it, rebuild, and re-source:

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

---

## alternative-teleop

Install:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Run:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Verify output with:

```bash
ros2 topic echo /cmd_vel
```

---

## reporting-issues

When troubleshooting, please include:

- The exact `ros2 launch` or `ros2 run` command used
    
- Full terminal output (especially `[ERROR]` lines)
    
- Output of:
    

```bash
ros2 topic echo /cmd_vel
```
