# Flyability Simulation Package
## Tested on (SubT Docker container)
ROS Melodic  
Ubuntu 18.04

## Prerequiste
Follow the [tutorial](https://bitbucket.org/osrf/subt/wiki/tutorials/SystemSetupDocker) from darpa to set up a container

## Using DARPA SubT Docker container
After opening the SubT container, run the following:

Clone this repository to the `~/subt_ws/src` directory, then.

```
cd ~/subt_ws
catkin_make
source ~/subt_ws/install/setup.bash
source ~/subt_ws/devel/setup.sh
rospack profile
sudo apt-get install ros-melodic-effort-controllers
```

To get the altimeter plugin we use the hector drone:
```
cd ~/subt_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git
cd ~/subt_ws
catkin_make
```

Launch the drone in an empty world:
```
roslaunch flya_gazebo elios_simulate.launch
```

To send a waypoint navigation command:
```
rostopic pub -1 /elios/waypoint flya_control/Waypoint "x: 0.0.0
y: 10.0
z: 9.0
yaw: 0.0"
```

The list of sensors and their corresponding topics are:

- IMU - `/elios/imu`
- Laser scans - `/flya/laser/scan`
- Altimeter - `elios/altimeter`
- Camera - `/elios/flya/front_camera` namespace
- Optical Flow - `/elios/flya/of*` * being is the camera number. Please note that optical flow algorithm is not given. Only the cameras are provided for you to write your algorithm.

The following actuators are available:

- Lights - `rosservice call /elios/<right/mid/left>_flashlight/enable "data: true"`
- LiDAR motor - `rostopic pub /elios/lidar_joint_position_controller/command std_msgs/Float64 "data: <speed in rads/sec>"`
