# Robot Design

## Bill of Materials
- The Robot
    - Motors: [Geared DC Motor, Adafruit #4416](https://www.adafruit.com/product/4416)
        - M3 Bolts?
    - Possible Sensors
        - https://store.stereolabs.com/products/zed-mini
        - https://shop.luxonis.com/collections/oak-cameras-1/products/oak-d-lite-1?variant=42583102456031
        - https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d415.html
- The Docking Station
- The Kiosk
- Controller

## Design Departments
For a more technical look at th edesign, check out the links below!

- [Mechanical Design](departments/mechanical.md)
- [Electrical Design](departments/electrical.md)
- [Control Design](departments/control.md)
- [Software Design](departments/software.md)


---
WORK IN PROGRESS
---

Don't forget to have fun. A vida boa.

General design Criteria here, then pages for specific goodies


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
    - Four Wheels
        - Two driven, two passive (castor)

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
