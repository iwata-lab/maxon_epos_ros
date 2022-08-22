# Maxon EPOS Package

This is a ROS/ROS2 package for Maxon motor & EPOS driver.

## Support Status
This package is **WIP** now.<br/>
v1.0.0 supports multiple EPOS device.<br/>

## How to use

### Install Maxon Linux driver
Firstly you must install maxon official linux driver.<br/>
For detail, please look at [this official pdf](http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos/EPOS_Command_Library.pdf)

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

- `~/<motor_name>/get_state` (msg: [maxon_epos_msgs/MotorState](maxon_epos_msgs/msg/MotorState.msg))

    This topic keep publishing motor current state.

- `~/<motor_name>/set_state` (msg: [maxon_epos_msgs/MotorState](maxon_epos_msgs/msg/MotorState.msg))

    When you send the goal to epos, use this topic.

- `~/get_all_state` (msg: [maxon_epos_msgs/MotorStates](maxon_epos_msgs/msg/MotorStates.msg))

    This topic keep publishing all motors current states.

- `~/set_all_state` (msg: [maxon_epos_msgs/MotorStates](maxon_epos_msgs/msg/MotorStates.msg))

    When you send the goals to all epos motors, use this topic.

#### Params

see [maxon_epos_example/config/example.yaml](maxon_epos_example/config/example.yaml)

### Example

see [maxon_epos_example/launch/example_maxon_epos.launch](maxon_epos_example/launch/example_maxon_epos.launch)

