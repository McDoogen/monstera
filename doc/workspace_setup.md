# Workspace Setup


## Setup [ROS Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
This project uses ROS 2 Jazzy. You can set that up by following the procedure documented in the link above.

To source your ROS2 Environment on every session, run the following command.

```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

We'll also install a few other dependencies needed for development with ROS.
```
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```


## Install [Micro ROS Agent](https://github.com/micro-ROS/micro_ros_setup?tab=readme-ov-file#building-micro-ros-agent)

1. Clone the Micro ROS repository into your Workspace. Run all of these commands from the project root directory
```
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
2. Build the Micro ROS repository for Jazzy
```
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```

3. Setup and Build the Micro ROS Agent
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent [parameters]
```

4. Now, whenever you want to run the agent, you can use the following command as an example.
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0
```


## Setup GPIO on the PI
#TODO:DS: Archive this... for referencing

This project will use the C++ bindings for the [libgpiod](https://github.com/brgl/libgpiod) library to control the GPIO of the Raspberry PI.

//TODO:DS: Nevermind, don't do this.... wrong version. Instead we will build from source in our CMakeLists.txt....

```
sudo apt install gpiod libgpiod-dev
```

To build... but how do we do this with CMakeLists and ament_cmake?
```
g++ -Wall -o gpio gpip.cpp -lgpiodcxx
```