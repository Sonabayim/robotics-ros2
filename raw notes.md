how to install ros 2 humble to ubuntu22.04 link the official documentation 
gazebo you need to explain classic and ignition fortress difference in installation and pros and cons

sourcing in each terminal detailed explanation on gazebo and rviz run commands

Install the ROS 2 and Gazebo‑ROS bridge packages
 sudo apt update 
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-gazebo-ros-pkgs

mkdir -p ~/turtlebot3_ws/src && cd $_
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install


## 2. Using Ignition Fortress

If you prefer Ignition (gazebo‑fortress) you can use the community “navigation2_ignition_gazebo_turtlebot3” package:

1. **Install Ignition‑ROS bridge & Nav2**
    
    ```bash
    sudo apt update
    sudo apt install \
      ros-humble-ros-ign-gazebo \
      ros-humble-ros-ign-bridge \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup
    ```
    
2. **Clone & build the Ignition‐ready TurtleBot3**
    
    ```bash
    mkdir -p ~/tb3_ign_ws/src && cd $_
    git clone https://github.com/Onicc/navigation2_ignition_gazebo_turtlebot3.git
    cd ~/tb3_ign_ws
    colcon build --symlink-install
    ```
    
3. **Source, set your model & host**
    
    ```bash
    source ~/tb3_ign_ws/install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    export ROS_LOCALHOST_ONLY=1
    ```
    
4. **Launch the Ignition simulation + Nav2 + RViz2**
    
    ```bash
    ros2 launch turtlebot3 simulation.launch.py
    ```
    
    That single command will spawn your TurtleBot3 in Ignition Fortress and bring up Navigation2 + RViz2 citeturn5view0.
    

---

### Tips

- If you hit missing‐package errors for `turtlebot3_gazebo`, make sure you’ve selected the **humble** branch when cloning or that your architecture (x86_64 vs. ARM) is supported by the binary packages citeturn7search0.
    
- You can always switch between Classic and Ignition by installing both sets of Gazebo packages and choosing the appropriate launch file.
    
- For keyboard‐teleop, open another terminal (source your workspace, set `TURTLEBOT3_MODEL`) and run:
    
    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```
    
