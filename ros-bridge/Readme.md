
# ROS bridge for CARLA simulator

This ROS package aims at providing a simple ROS bridge for CARLA simulator.

__Important Note:__
This documentation is for CARLA versions *newer* than 0.9.1. The CARLA release 0.9.1
does not work out of the box with the ROS bridge.

![rviz setup](./assets/rviz_carla_default.png "rviz")
![depthcloud](./assets/depth_cloud_and_lidar.png "depthcloud")

![short video](https://youtu.be/S_NoN2GBtdY)


# Features

- [x] Cameras (depth, segmentation, rgb) support
- [x] Transform publications
- [x] Manual control using ackermann msg
- [x] Handle ROS dependencies
- [x] Marker/bounding box messages for cars/pedestrian
- [x] Lidar sensor support
- [ ] Rosbag in the bridge (in order to avoid rosbag record -a small time errors)
- [ ] Add traffic light support

# Setup

## Create a catkin workspace and install carla_ros_bridge package

First, clone or download the carla_ros_bridge, for example into

   ~/carla_ros_bridge

### Create the catkin workspace:

    mkdir -p ~/ros/catkin_ws_for_carla/src
    cd ~/ros/catkin_ws_for_carla/src
    cp ~/carla_ros_bridge/* .
    source /opt/ros/kinetic/setup.bash
    catkin_make
    source ~/ros/catkin_ws_for_carla/devel/setup.bash

For more information about configuring a ROS environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Install the CARLA Python API

    export PYTHONPATH=$PYTHONPATH:<path/to/carla/>/PythonAPI/<your_egg_file>

Please note that you have to put in the complete path to the egg-file including
the egg-file itself. Please use the one, that is supported by your Python version.
Depending on the type of CARLA (pre-build, or build from source), the egg files
are typically located either directly in the PythonAPI folder or in PythonAPI/dist.

Check the installation is successfull by trying to import carla from python:

    python -c 'import carla;print("Success")'

You should see the Success message without any errors.

## Install other requirements:

    sudo apt-get install python-protobuf
    pip install --user simple-pid


# Start the ROS bridge

First run the simulator (see carla documentation: http://carla.readthedocs.io/en/latest/)

    ./CarlaUE4.sh -windowed -ResX=320 -ResY=240 -benchmark -fps=10


Wait for the message:

    Waiting for the client to connect...

Then start the ros bridge:

    source ~/ros/catkin_ws_for_carla/devel/setup.bash
    roslaunch carla_ros_bridge client.launch

To start the ros bridge with rviz use:

    source ~/ros/catkin_ws_for_carla/devel/setup.bash
    roslaunch carla_ros_bridge client_with_rviz.launch

You can setup the vehicle configuration config/settings.yaml.

As we have not spawned any vehicle and have not added any sensors in our carla world there would not be any stream of data yet.

You can make use of the CARLA Python API script manual_control.py.
```
cd <path/to/carla/>
python manual_control.py
```
This spawns a vehicle with role_name='hero' which is interpreted as the ego
vehicle as defined by the config/settings.yaml.

You can then further spawn other vehicles using spawn_npc.py from CARLA Python API.
Then those vehicles will show up also on ROS side.

# Test control messages
You can send command to the car using the /carla/ego_vehicle/ackermann_cmd topic.

Example of forward movements, speed in in meters/sec.

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10


Example of forward with steering

     rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 5.41, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0,
      jerk: 0.0}" -r 10

  Warning: the steering_angle is the driving angle (in radians) not the wheel angle, for now max wheel is set to 500 degrees.

# Test sensor messages

## Object information

### Ego vehicle

The ego vehicle status is provided via the topic /carla/ego_vehicle/odometry (nav_msgs.Odometry)

### Other vehicles

The other vehicles data is provided via the 'ideal' object list /carla/objects (derived_object_msgs.ObjectArray)

## Map information

The OPEN Drive map description is published via /map topic (std_msgs.String)

## Sensor information

### Ego vehicle
The ego Vehicle sensors are provided via topics with prefix /carla/ego_vehicle/<sensor_topic>


# ROSBAG recording (not yet tested)

The carla_ros_bridge could also be used to record all published topics into a rosbag:

    roslaunch carla_ros_bridge client_with_rviz.launch rosbag_fname:=/tmp/save_session.bag

This command will create a rosbag /tmp/save_session.bag

You can of course also use rosbag record to do the same, but using the ros_bridge to do the recording you have the guarentee that all the message are saved without small desynchronization that could occurs when using *rosbag record* in an other process.
