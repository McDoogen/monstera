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

### Blinky
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
- [ament_cmake](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html)

### Tutorials
- [cmake](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)

### Relevant ROS Enhancement Proposals (REPs)
- [REP 103 - Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP 105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

---
WORK IN PROGRESS
---

## Research
- https://en.wikipedia.org/wiki/Control_theory
- https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
- Controllers
- PID Controller
- Model Predictive Control
- Fuzzy Logic Control
- Neural Network Control


## TODO
- Convert the 3D model into a URDF to use in Gazebo
- Do I need individual licenses on each package too?
- Documentation here is for the workspace. How to build and run the packages, and how to create a new package.
- Look into ROS 2 overlays & underlays
- Add a cheat sheet of commands, like rosdep install --from-paths src -y --ignore-src. And include the creating a new package one!
- Start a Glossary? Odometry?
- Troubleshooting page with export QT_QPA_PLATFORM=xcb rviz2? [See this link](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html)
- URDF- Geometry, Model, Physical Properties. Start with the geometry for the transformationa
- Source setup or local_setup? https://colcon.readthedocs.io/en/released/developer/environment.html#workspace-level
- We should include the logic shifter in the documentation. 2 3.7V = 7.4V. We need 3.3V for logic. https://www.adafruit.com/product/2165