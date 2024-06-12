# Useful Pi Pico Documentation

- [Getting Started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [Pi Pico Pinout Diagram](https://pico.pinout.xyz/)



# Setup for development on Linux!
## Setup Pico Development Workspace with [Pico-SDK](https://github.com/raspberrypi/pico-sdk)

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



## Setup [Picotool](https://github.com/raspberrypi/picotool)

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



## Setup ROS Jazzy
This project uses ROS 2 Jazzy. You can set that up by following the procedure documented [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

And to source your ROS2 Environment on every session, run this command

```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## Setup micro-ros-agent

From the project root directory, run the following commands
```
mkdir uros_ws && cd uros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
```


### Sources
[Source for building micro_ros](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy?tab=readme-ov-file#building)

[Source for building micro_ros_agent](https://github.com/micro-ROS/micro_ros_setup/tree/jazzy?tab=readme-ov-file#building-micro-ros-agent)

#TODO:DS: Do I need to do this all in a ROS2 workspace? Maybe I should start there...
#TODO:DS: Should we change the directory where this is installed? Can we automagically include this in the CMakeLists?
#TODO:DS: clean up this section
#TODO:DS: do we need to source the local_setup every time? Should we add this to the bashrc?
#TODO:DS: Read again how the workspaces work
#TODO:DS: Example
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0
```

#TODO:DS: Redo this for our project. Simplify the steps from the repo and specify it for jazzy. Or should we just use the docker container?


## TODO: What else needs to be set up?
- Loading the uf2, monitoring the PI PICO minicom -b 115200 -o -D /dev/ttyACM0
- How do I use that debugger probe too?
- micro-ros-agent? https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico


# Build this project

3. Select the **GCC arm-none-eabi** kit under Configure, and Build using the CMake extension. Voila!



# TODO
- ros2 topic pub --once /pico_subscriber std_msgs/msg/Int32 '{"data":100}'

