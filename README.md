# Maxon EPOS Package

This is a ROS/ROS2 package for Maxon motor & EPOS driver.

## Support Status
This package is **WIP** now.<br/>
v1.0.0 supports multiple EPOS device.<br/>

## How to use
> **WARNING** --- The package maxon_epos_example **has NOT** been migrated to ROS2.</br>

In the package maxon_epos_driver you can find an example of launch file and configuration file for runing one motor. </br>

> **VERY IMPORTANT** - Do not set the name of the node when using a launch file to execute the maxon_bringup node. This will rename all rclcpp::Node variables that it node generates and it will be unable to get the motor connection parameters correctly.

### Install Maxon Linux driver
Firstly you must install maxon official linux driver. [Download EPOS Linux Library here](https://www.maxongroup.com/medias/sys_master/root/8994700394526/EPOS-Linux-Library-En.zip)<br/>
For detail, please look at [this official pdf](https://www.maxongroup.com/medias/sys_master/8823917281310.pdf)

### How to build
Then you have to download this package into your ROS workspace.
```bash
$ cd ~/yourworkspace_ws/src
$ git clone https://github.com/cristinaluna/maxon_epos_ros.git
$ cd ~/yourworkspace_ws
$ colcon build 
$ source ~/yourworkspace_ws/install/setup.bash
```
### How to use nodes

#### Run node
You can run nodes by console.<br/>
`maxon_bringup` node reads params from ROS parameter server.
```bash
$ ros2 run maxon_epos_driver maxon_bringup
```

Otherwise, you can refer `maxon_epos_example` package's launch file.

#### Topics

- `~/<motor_name>/get_state` (msg: [maxon_epos_msgs/msg/MotorState](maxon_epos_msgs/msg/MotorState.msg))

    This topic keep publishing motor current state.

- `~/<motor_name>/set_state` (msg: [maxon_epos_msgs/msg/MotorState](maxon_epos_msgs/msg/MotorState.msg))

    When you send the goal to epos, use this topic.

- `~/get_all_state` (msg: [maxon_epos_msgs/msg/MotorStates](maxon_epos_msgs/msg/MotorStates.msg))

    This topic keep publishing all motors current states.

- `~/set_all_state` (msg: [maxon_epos_msgs/msg/MotorStates](maxon_epos_msgs/msg/MotorStates.msg))

    When you send the goals to all epos motors, use this topic.

#### Params

see [maxon_epos_example/config/example.yaml](maxon_epos_example/config/example.yaml)

### Example

see [maxon_epos_example/launch/example_maxon_epos.launch](maxon_epos_example/launch/example_maxon_epos.launch)

