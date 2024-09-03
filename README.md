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

### Monstera Blinky
This package is a 'Hello World' package that I will use while setting up my development workflow. It will consists of a Node subscribed to teleop messages and blink different LEDs accordingly.

### Monstera Main Controller
TODO:DS: Brief detail on Package 1
TODO:DS: How to build Package 1
TODO:DS: How to deploy Package 1
TODO:DS: Link to Package 1, for more details and specific component documentation

### Monstera Motor Teleoperation
#TODO:DS: First phase of control, just basic teleoperation. 

### Creating a New Package
To create a new package, run the following command from the workspace **src** directory:

```
ros2 pkg create --build-type ament_cmake --license MIT <package_name>
```

The build type can be ament_cmake, cmake, or python_cmake. You can read more about creating ROS 2 packages [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package).

## 3rd Party Packages
- [teleop_twist_keyboard](https://index.ros.org/r/teleop_twist_keyboard/#jazzy)


## Sources
TODO:DS: List your sources and links for reseearch etc

### Useful Documentation
- [Getting Started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [Pi Pico Pinout Diagram](https://pico.pinout.xyz/)

### Tutorials
- [cmake](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)

### Relevant ROS Enhancement Proposals (REPs)
- [REP 103 - Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP 105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

---
WORK IN PROGRESS
---

Links for ko-fi, etc



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
- Should I install the pico-sdk source local to this workspace instead? Is that what the include directory is for? Maybe this will help: https://answers.ros.org/question/364293/how-to-include-library-header-files/
- Look into ROS 2 overlays & underlays
- Can I have cmake build and install micro_ros_setup? And then I just need to create the agent?
- How do I supress warnings on dependencies? SYSTEM?
- Add a cheat sheet of commands, like rosdep install --from-paths src -y --ignore-src. And include the creating a new package one!
- Start a Glossary? Odometry?
- Troubleshooting page with export QT_QPA_PLATFORM=xcb rviz2? [See this link](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html)
- URDF- Geometry, Model, Physical Properties. Start with the geometry for the transformationa
- Clean out the PICO parts
- Figure out GPIO with the onboard PI control.
- How do I deploy this to my Robot in the most straight-forward manner? Container?
- How do I deploy and install my ROS2 packages to the Robot without building it on the Robot? Can I creat an apt package? A Snap? a rosdep package thing?
- Can I pre-install apt dependencies on an Ubuntu build? How can I make a custom Ubuntu build?
- What's a goof GPIO hello world? A pub, a Sub, and a blinking light. One package, two nodes. Or a pub and teleop with a blinking light?
- Move component documents into the src folder with the component