KHI ROS2 Driver
===================================================================================================================================================

This repository provides ROS2 support for KHI robots.  
ROS2 distribution `Humble` is supported.

## Supported system

### Supported Robot Controller Software Version
Controller(F01A/F02A): Higher than ```ASF_01010200E```  
Controller(F01B/F02B/F60B): Higher than ```ASF_01040200A```  
Controller(F60A): Higher than ```ASF_010000020```

### Supported Robot

 * RS007L-B001
 * RS013N-A001
 * RS015X-A001
 * RS025N-A001
 * RS080N-A001
 * BX300L-B001
 * BXP135X-A001

## Requirements

### Execution Environment
Make sure that the Ubuntu PC used for real-time control satisfies the following conditions.

* The PC is using realtime kernel for Ubuntu 22.04.
* The user has real-time permissions.
  * (e.g.)Making a real-time group named `realtime`
    1. Make a group and add a user

       ```
       sudo addgroup realtime
       sudo usermod -aG realtime $(whoami)
       ```

    2. Add the following limits in `/etc/security/limits.conf`  
       (`memlock` depends on your system)

       ```
       @realtime - rtprio 99
       @realtime - priority 99
       @realtime - memlock 512000
       ```

If real-time support is not properly configured, a warning saying 'Could not enable FIFO RT scheduling policy' will be displayed when starting real-time control.

## Packages in the Repository

- `khi_description` Contains configuration files for each KHI robot.
- `khi_gazebo` Contains configuration and launch files for simulating KHI robots in Gazebo Classic.
- `khi_hardware` Contains configuration and launch files for operating KHI robots on actual hardware.
- `khi_moveit` Contains configuration and launch files for operating KHI robots with Moveit!.
- `khi_msgs` Contains definition files for KHI-specific services and messages.

## Lifecycle

[khi_hardware](khi_hardware/docs/lifecycle.md)

## Services and Publishers

[khi_msgs](khi_msgs/README.md)

## Preparation

Make sure that the robot controller used for real-time control satisfies the following conditions.

* The controller is connected with a robot and ready to operate.
* Nobody is inside the safety fence.
* The controller is in [REPEAT] mode.
* TEACH LOCK on the Teach pendant is switched to OFF.
* Robot is not on Hold status.
* No error.
* The controller is connected to an Ubuntu PC with Ethernet cable and both are within the same network subnet.

Set the load mass and load center of gravity for the tool mounted on the robot.
```
>weight
   WEIGHT          X         Y         Z  Inertia X  Inertia Y  Inertia Z
      7.00      0.00      0.00      0.00      0.00      0.00      0.00
Change? (If not, Press RETURN only.)
```

## How to Launch

### (1) Launch Control Node

Start ```khi_hardware``` as:  
```
ros2 launch khi_hardware khi_bringup.launch.py robot:=<robot_name> robot_ip:=<robot_ip_address>
```
(e.g.) `ros2 launch khi_hardware khi_bringup.launch.py robot:=rs080n-a001 robot_ip:=192.168.0.3`

When using Gazebo Classic:  
```
ros2 launch khi_gazebo khi_gazebo.launch.py robot:=<robot_name>
```

### (2) Launch MoveIt! Node

Start a MoveIt! script as:  
```
ros2 launch khi_moveit khi_moveit.launch.py robot:=<robot_name>
```

When using Gazebo Classic:  
```
ros2 launch khi_moveit khi_moveit.launch.py robot:=<robot_name> use_gazebo:=true use_sim_time:=true
```

### Arguments
Please refer to the following for details on the arguments.  
- [khi_bringup.launch.py](khi_hardware/launch/khi_bringup.launch.py)
- [khi_gazebo.launch.py](khi_gazebo/launch/khi_gazebo.launch.py)
- [khi_moveit.launch.py](khi_moveit/launch/khi_moveit.launch.py)

## Error and Troubleshooting

[troubleshooting](khi_hardware/docs/troubleshooting.md)

## Precausions

* Make sure to use realtime kernel for Ubuntu 20.04/22.04.
* When a robot controller is in the real-time control mode, its state is same as the REPEAT mode. Therefore make sure to the safety issues when the robot controller is in real-time control mode.
* Never make any changes on the sources of the “khi_ros2” package.
* Refer to the Documents and community of the MoveIt! for more details on motion/path planning and how to calculate command value.
* Connecting multiple processes to the same controller is not supported.

## Notes

### About this software

The APIs are completely unstable and likely to change. Use in production systems is not recommended.

### About Coordinate

KHI coordinate and ROS cordinate are different.  
Origin of KHI coordinate is Robot Link1 origin.  
Origin of ROS coordinate is World origin.

### About CAD data

`khi_description` are using STL files based on CAD Data of the KHI website.  
Therefore [KHI CAD Data Disclaimer](https://robotics.kawasaki.com/en1/products/CAD-disclaimer/?language_id=1) is also applied to these files.

### About License
This software includes multiple components, which may be subject to different licenses.  
For details, please refer to the LICENSE file.
