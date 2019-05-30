# Maxon EPOS Package

This is a ROS package for Maxon motor & EPOS driver.

## Support Status
This package is **WIP** now.<br/>
v0.0.0 supports only one EPOS device.

## How to use

### Install Maxon Linux driver
Firstly you must install maxon official linux driver.<br/>
For detail, please look at [this official pdf](http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos/EPOS_Command_Library.pdf)

### How to build
Then you have to download this package into your ROS workspace.
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/iwata-lab/maxon_epos_ros.git
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```
### How to use nodes

#### Run node
You can run nodes by console.<br/>
`maxon_bringup` node reads params from ROS parameter server.
```bash
$ rosrun maxon_epos_driver maxon_bringup
```

Otherwise, you can refer `maxon_epos_example` package's launch file.

#### Params

`device` (string, default: "EPOS4")

- type of device name such as "EPOS", "EPOS2", or "EPOS4"

`protocol` (string, default: "MAXON SERIAL V2")

- type of protocol stack such as "MAXON RS232", "MAXON SERIAL V2", or "CANopen"

`interface` (string, default: "USB")

- type of interface such as "USB" or "RS232"

`port` (string, default: "USB0")

- port name for connection such as
    - "COM0", "COM1", ....
    - "USB0", "USB1", ....
    - "CAN0", "CAN1", ....

`baudrate` (int, default: 1000000)

- baudrate of communication via phisical interface


### Example

see [maxon_epos_example/launch](maxon_epos_example/launch)

