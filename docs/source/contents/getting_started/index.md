# Getting Started

```{warning}
This project is currently in progress.
```

This guide will show you all the necessary installation steps required to use CMPF for developing and testing self-driving applications in simulation.

## Installation

CMPF is based on [Robot Operating System (ROS)](https://www.ros.org/) and uses [CARLA](https://carla.org/) as a main autonomous driving simulator. Before installing CMPF, make sure you have already installed the latest version of ROS and CARLA. Additionally, CMPF relies on CARLA ros-bridge for the communication between ROS and CARLA simulator. You can follow the [CARLA ros-bridge documention](https://carla.readthedocs.io/projects/ros-bridge/en/latest/) for installing the bridge from the Debian repository or from source.

```{important}
CMPF is currently only supported for ROS 1. Make sure you have installed [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [CARLA ROS bridge for ROS 1](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/) before building CMPF.
```

### Building CMPF from the source repository

Make sure you have already set up the ROS environment by sourcing the `/opt/ros/noetic/setup.bash` file.

__1.__ Create a catkin workspace:
```sh
    mkdir -p ~/cmpf-ros/catkin_ws/src
```

__2.__ Clone the CMPF source repository:
```sh
    cd ~/cmpf-ros/catkin_ws/src
    git clone https://github.com/mlsdpk/cmpf.git
```

__3.__ Install the required ros-dependencies:
```sh
    cd ..
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro noetic
```

__4.__ Build the CMPF:
```sh
    catkin_make
```