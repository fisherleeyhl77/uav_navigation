## State UAV Navigation with Asctec Firefly and Realsense R200 Camera


This repository implements the 3D Localisation, Mapping and reactive Pathplanning/Pathfollowing of the AscTec Firefly with a mounted Realsense R200 Camera.

- For the 3D Localisation we create position data using extended kalman filtered signals from the firefly-IMU(inertial measurement unit) and camera-VO(visual odometry). The IMU is already built-in the Firefly. Visual odometry is created from the realsense R200 camera.

- 3D Mapping was first implemented with octomap. Later in the final version it was upgraded to RTAB-Map (Real-Time Appearance-Based Mapping) which has more features. Any other similar occupancy grid mappings would also work. Similar other depth cameras than realsense r200 would also work.

- The reactive Pathplanning is implemented with the OMPL(Open Motion Planning Library) in Moveit!(Motion Planning Framework) using the real-time generated maps from RTAB-Map. 

- For the Pathfollowing we use the generated obstacle avoiding paths and create custom controller signals the Asctec Firefly.



### Installation Instructions for Ubuntu 16.04 with ROS Kinetic


1. Install and initialize ROS kinetic desktop full.

2. Install additional packages:

```
$ sudo apt-get install ros-kinetic-moveit ros-kinetic-rtabmap-ros ros-kinetic-mavlink ros-kinetic-robot-pose-ekf ros-kinetic-octomap-ros ros-kinetic-joy python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
```

3. Install Realsense R200 camera by following instructions on:

http://wiki.ros.org/realsense_camera

4. Create and build catkin workspace with the project:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/nihsal/uav_navigation.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ cd ~/catkin_ws/
$ catkin build
```

In case of "error no module future" run:

```
$ sudo apt-get install python-pip
$ pip install future
```

5. Add sourcing to your `~/.bashrc` file:

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc 
```

### Usage Instructions for simulated environment

```
$ roslaunch uav_navigation_bringup firefly_and_obstacle_simulation.launch
``` 

### Usage Instructions for real environment

1. Connect the r200 camera and launch:

```
$ roslaunch uav_navigation_bringup uav_bringup.launch 

``` 

