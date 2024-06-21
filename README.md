# Monstera: Cat Finding Robot
TODO:DS: What is Monstera? What is the purpose of this project?

## Robot Design
TODO:DS: Briefly mention it here

[Link to Document](doc/robot_design.md)

## Setting up your workspace
TODO:DS: Using Ubuntu 24, using ROS2 Jazzy. Environment requirements?  Keep it brief.

[Link to Document](doc/workspace_setup.md)

## Packages
TODO:DS: brief detail on how to build the overall package
TODO:DS: Information on how to build each individual package

### Package 1
TODO:DS: Brief detail on Package 1
TODO:DS: How to build Package 1
TODO:DS: How to deploy Package 1
TODO:DS: Link to Package 1, for more details and specific component documentation

## Sources
TODO:DS: List your sources and links for reseearch etc


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
- Add links to sub-documents
- I think I need to split this project into different repos? Let's spend some time thinking of the project structure using PICO & micro-ros + ROS + design documentation.
- Add steps on how packages were created?
- Add the build steps... colcon etc
- Add more steps for setting up workspace? Still need to think how to organize the repo....
- Do I need individual licenses on each package too?
- Can I combine the ROS and micro-ros into one package with ament_cmake?
- Layout: Separate Packages and Repos for each package. Instructions on how to setup the workspace, and which packages to include Maybe a repo for setting up the workspace?
- Documentation here is for the workspace. How to build and run the packages, and how to create a new package.
- Should we put documentation for the specific tools like Pi Pico in the individual packages or here? Whats the idea here? This Workspace is for building these two specific package for running on the Monstera Robot. Sure, they can split off. But for this case they are tightly coupled to the Monstera Robot
- So should all documentation be in the workspace?
- Make sure to include plenty of links for pi pico development and ROS2 etc