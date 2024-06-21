# Workspace Setup

TODO:DS: Here I will detail setting up your workspace for development of this group of packages.

---
TODO
---



### Installing Dependencies
TODO:DS: ...

### Setting up a new package
This project consists of ROS 2 packages that compliment the major components of the robot. To create a new package, run the following command from the workspace src directory:

```
ros2 pkg create --build-type ament_cmake --license MIT <package_name>
```

The build type can be ament_cmake, cmake, or python_cmake. You can read more about creating ROS 2 packages [here](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package).