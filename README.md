# Monstera: Cat Finding Robot
Monstera is a robot that can autonomously navigate a home identifying cats.

## Robot Design
Monstera is a four wheeled robot with a depth camera. The camera will provide navigation for the robot, the ability to identify cats, as well as provide live video during teleoperation. Check the link below for bore details on the design of this robot.

[Robot Design](doc/robot_design.md)

## Setting up your workspace
This project provides instructions for building and installing software components from Ubuntu 24 with ROS 2 Jazzy. Check the link beflow for instructions on setting up your development environment.

[Workspace Setup](doc/workspace_setup.md)

## Packages
This project consists of ROS 2 packages that compliment the major components of the robot. 
TODO:DS: brief detail on how to build the overall package

### Monstera Main Controller
TODO:DS: Brief detail on Package 1
TODO:DS: How to build Package 1
TODO:DS: How to deploy Package 1
TODO:DS: Link to Package 1, for more details and specific component documentation

### Monstera Motor Controller
TODO:DS: ...

### Creating a New Package
To create a new package, run the following command from the workspace **src** directory:

```
ros2 pkg create --build-type ament_cmake --license MIT <package_name>
```

The build type can be ament_cmake, cmake, or python_cmake. You can read more about creating ROS 2 packages [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package).

## Sources
TODO:DS: List your sources and links for reseearch etc

### Useful Tutorials
- [cmake](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)


---
WORK IN PROGRESS
---

I think I need to better separate Robot Design from software components. Treat software packages as components to the robot design. Design first, software second.

Links for ko-fi, etc


## What is the Objective of this Robot?
The objective of this robot is to explore indoor areas to find your cat. This is good for when you are not at home but want to keep an eye on your pet.




## Research
- https://en.wikipedia.org/wiki/Control_theory
- https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
- Controllers
- PID Controller
- Model Predictive Control
- Fuzzy Logic Control
- Neural Network Control

## Test Setup
### PICO Wiring
```
              |GP00|-----|VBUS|
              |GP01|-----|VSYS|
  Bread (GND) |GND |-----|GND | IMU(GND)
   Motor2 (A) |GP02|-----|3V3E|
   Motor2 (B) |GP03|-----|3V3 | IMU(3.3)
   Motor1 (A) |GP04|-----|ADCR|
   Motor1 (B) |GP05|-----|GP28|
              |GND |-----|ADCG|
Motor1 (PWMA) |GP06|-----|GP27|
Motor1 (AIN2) |GP07|-----|GP26|
Motor1 (AIN1) |GP08|-----|RUN |
Motor2 (BIN1) |GP09|-----|GP22|
 Bridge (GND) |GND |-----|GND |
Motor2 (BIN2) |GP10|-----|GP21| IMU(X)
Motor2 (PWMB) |GP11|-----|GP20| IMU(Y)
              |GP12|-----|GP19| IMU(Z)
              |GP13|-----|GP18| IMU(ST)
              |GND |-----|GND |
              |GP14|-----|GP17|
              |GP15|-----|GP16|
```

## TODO
- Gather and design the electrical components
- Gather and design the physical components
- Start to make 3D models (onshape)
    - All on one level? Sensors and motors. This helps with consistensy. Multiple levels may have some rotation due to inaccuracies of the standoffs.
- Convert the 3D model into a URDF to use in Gazebo
- Do I need individual licenses on each package too?
- Documentation here is for the workspace. How to build and run the packages, and how to create a new package.
- Should we put documentation for the specific tools like Pi Pico in the individual packages or here? Whats the idea here? This Workspace is for building these two specific package for running on the Monstera Robot. Sure, they can split off. But for this case they are tightly coupled to the Monstera Robot
- So should all documentation be in the workspace?
- Make sure to include plenty of links for pi pico development and ROS2 etc
- Should I install the pico-sdk source local to this workspace instead? Is that what the include directory is for? Maybe this will help: https://answers.ros.org/question/364293/how-to-include-library-header-files/
- Building the micro-ros package will colcon with a cmake configured package. It will skip over micro_ros_setup and use [this library](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)