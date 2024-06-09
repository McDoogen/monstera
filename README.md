# Monstera: Cat Finding Robot
## What is the Objective of this Robot?
The objective of this robot is to explore indoor areas to find your cat. This is good for when you are not at home but want to keep an eye on your pet.

## What are the Design Criteria of this Robot?
- Navigate a single floor indoors autonomously.
- Identify cats
- Navigate autonomously
- Dock and recharge
- Allow user control
    - Kiosk (Linux Server Application)
    - Ubuntu Frame

## What will the Design of the Robot be?
- Locomotion
    - Three wheels?
        - Two driven, one passive
    - Four Wheels?
        - Two driven, two passive (castor)
        - Four driven

## Design Procedure
1. Determine Robot Workplace
    - Environment
1. Determine Robot Locomotion
    - Wheels
        - Differential Drive
            - https://en.wikipedia.org/wiki/Differential_wheeled_robot 
    - Legged
    - etc
1. Model the System
    - Equations of Motion
        - Kinematic
        - Dynamics
    - Get that model that includes the localization and path planning
        - What is the name of this? Is it a block diagram?
        - How does this fit into the overall picture?
1. Determine the Control Characteristics
    - Read through https://en.wikipedia.org/wiki/Control_theory, for an overview.
    - Linear or non-linear
    - SISO vs MIMO
1. Determine the Control System for the Model
    - Read through https://en.wikipedia.org/wiki/Control_theory, for an overview.
    - PID vs MPC vs other
1. Determine what tools will help with development	
    - Python (numpy, scipy, pandas, pytorch, etc)
    - ROS
    - MoveIt
    - Nav2
    - Ros2_control
    - Gazeo
    - Determine the PhysicalComponents that fit the Controller
    - Processor
    - Sensors
    - Actuators

## Bill of Materials
- The Robot
    - Motors: [Geared DC Motor, Adafruit #4416](https://www.adafruit.com/product/4416)
        - M3 Bolts?
- The Docking Station
- The Kiosk
- Controller

### Motor Notes
- 2 M3 bolt holes on the front surface. 17mm apart
- 3mm diameter shaft, 11mm long, keyed .45mm deep
- For the wheel... maybe we can laser cut an acrylic wheel, when attach our own rubber to it... That way its a perfect fit! Or maybe 3D print the wheel! :D
- Swivel wheel option: [Adafruit #2942](https://www.adafruit.com/product/2942)


## Notes
- Create other Repos for tutorials like using the PI PICO, or for building individual components?
- Your Tools
    - Start to classify and organize all your tools you will use
        - Mathematical Tools
            - Jacobian 
            - DH Thing 
            - Kalman Filter 
            - Numerical Methos vs _? 
            - Model Order Reduction 
                - Reduce states in a System’s Dynamics ? 
            - Controllability 
            - Observability 
        - Controllers
            - PID Controller
            - MPC Controller
                - Linear time-invariant MPC ?
                    - Linear system, Linear Constraints, Quadratic cost function
                    - Convex Optimizatoin Problem ?
                - Linear MPC (Adaptive MPC, Gain-scheduled MPC) ?
                    - Nonlinear system, Linear Constriants, Quadratic cost function
                    - Convex Optimizatoin Problem ?
                - Nonlinear MPC
                    - Nonlinear System, nonlinear constraints, nonlinear cost function
                    - Non-convex Optimization Problem
                    - Requires higher computation
                - Traditional MPC vs Explicit MPC
                - Suboptimal Solution
        - Software Tools
            - Numpy
            - ROS
            - SciPy
        - Other Tools
            - Behavior Trees
- Sources
    - Matlab videos?
- Things to Consider
    - Locomotion
    - Kinematics & Dynamics
        - Center of Gravity + Pivot Point
        - What prevents the robot from falling?
        - What are our control variables?
        - How does the robot correct itself from falling?
        - How does rhte robot simultaneously move and not fall?
    - Control Scheme
        - So what are we trying to control? And how are we controlling it? What are all our inputs and outputs? What is the ‘System’ ?
        - https://en.wikipedia.org/wiki/Inverted_pendulum
        - What tools can we use to solve these equations and problems?
            - Numpy
            - Scipy
            - Pandas
            - Pytorch
    - ROS Packages
    - Copy that chart of the cycle with perception
    - How do the mathematical concepts relate to ROS Packages?
    - Block Diagram for the Robot Components.
    - Make a write-up of how to solve this problem, of how each value and equation is determined.
    - Mathematical Concepts
        - https://en.wikipedia.org/wiki/Lagrangian_mechanics
        - https://en.wikipedia.org/wiki/Hamiltonian_mechanics
        - https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
        - https://en.wikipedia.org/wiki/Eigenvalues_and_eigenvectors
        - https://en.wikipedia.org/wiki/Nyquist_stability_criterion
        - https://en.wikipedia.org/wiki/Zeros_and_poles
        - https://en.wikipedia.org/wiki/Equations_of_motion
        - https://en.wikipedia.org/wiki/State-space_representation

## Phases (See other notes)
Balance in one place
Move forward and back
Turn left and right
User Control to navigate
Automated Control with Nav2

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
