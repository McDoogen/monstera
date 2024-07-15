# Workspace Setup

## Setup ROS Jazzy
This project uses ROS 2 Jazzy. You can set that up by following the procedure documented [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

To source your ROS2 Environment on every session, run the following command.

```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

We'll also install a few other dependencies needed for development with ROS.
```
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```




## Setup VSCode

1. Install VSCode

```
sudo apt install code
```

2. Install VSCode Extensions

```
code --install-extension marus25.cortex-debug code --install-extension ms-vscode.cmake-tools code --install-extension ms-vscode.cpptools
```

> Note: If you are using VSCode's "Remote Explore" to code over SSH, then make sure to reboot after setting up environment variables otherwise CMake will not be able to locate PICO_SDK_PATH

> Note: For more information about configuring VSCode for ROS 2, check out this article, [here](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)!


## Install [Pico-SDK](https://github.com/raspberrypi/pico-sdk)

1. Install the Toolchain

```
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

2. Clone the pico-sdk repository into your home directory

```
git clone https://github.com/raspberrypi/pico-sdk.git --branch master ~/pico-sdk
cd ~/pico-sdk
git submodule update --init
```

3. Add PICO_SDK_PATH to your bashrc

```
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```


## Install [Picotool](https://github.com/raspberrypi/picotool)

1. Install dependencies

```
sudo apt install build-essential pkg-config libusb-1.0-0-dev cmake
```

2. Clone picotool into your home directory

```
git clone git@github.com:raspberrypi/picotool.git ~/picotool
cd ~/picotool
```

3. Build Picotool!

```
mkdir build
cd build
cmake ../
make
```

4. Copy the CLI binary into your local binaries directory

```
sudo cp ~/picotool/build/picotool /usr/local/bin/
```

5. Now you can run picotool from any terminal!
```
sudo picotool info
sudo picotool load build/src/hello/hello.uf2
```

6. You can also remove the picotool repository from your home directory. We don't need that anymore.

```
rm -rf ~/picotool
```


## Setup micro-ros-agent

First, we need to install the micro_ros_setup package. From the workspace root directory, run the following commands
```
mkdir uros_ws && cd uros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```

Now we need to create the agent. You can do so by running the following commands from the workspace root directoy.
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

Now with the agent installed, you can start the agent with the following command.
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0
```

You can read more about the micro_ros_setup package as well as the agent at the following link.
[micro_ros_setup ](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy?tab=readme-ov-file#building)

---
WORK IN PROGRESS
---

## TODO: What else needs to be set up?
- Loading the uf2, monitoring the PI PICO minicom -b 115200 -o -D /dev/ttyACM0
- How do I use that debugger probe too?
- micro-ros-agent? https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico


## Build this project with VSCode

1. Select the **GCC arm-none-eabi** kit under Configure, and Build using the CMake extension. Voila!
#TODO:DS: Move build instructions to the individual package documents


# TODO
- ros2 topic pub --once /pico_subscriber std_msgs/msg/Int32 '{"data":100}'

