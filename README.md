power_tools
===========

Message definitions and hardware drivers for power supply devices using ROS

*While this code is freely licensed (2-clause BSD), I do ask that you send [me](mailto:calder.pg@gmail.com) an email if possible so I can see who is using this software.*

Repository structure
--------------------
Unlike earlier Catkinized software we have provided, this repository does not contain a Catkin workspace. As we expect that other teams will be well on their way to migrating to ROS Groovy, the difficulties of managing multiple workspaces do not justify the convenience of distributing these packages in their own workspace. As such, you will need to clone this repository inside the `src/` directory of an existing Catkin workspace.

Please note that this software is structured for ROS Groovy+, and is incompatible with ROS Fuerte and earlier.

This repository is structured around 3 core packages:

1.  `power_msgs` - Message and serive definitions for power devices. The PowerState message is designed to be suitable for all types of power devices, ranging from DC-DC converters to automotive power supplies to even laptop batteries.

2.  `dcdc_driver` - This package provides a ROS driver for the Mini-Box DCDC programable dc-dc converter.

3.  `dbus_power_monitor` - This package provides a monitor for the UPower daemon, which provides the "standard" interface to system power state on Linux desktop systems. For example, this allows a robot using a laptop to monitor the state of the laptop's battery(s).

Stability and development status
--------------------------------
`power_msgs` - This package is currently stable. Additional fields in the PowerState message are not currently planned. Additional service may be defined, but they will no break compatibility with the current interface.

`dcdc_driver` - Currently stable, although better error recovery will be added in the near future. Output voltage control will also be added in the comming months.

`dbus_power_monitor` - Currently stable, will be updated to maintain compatibility with dbus and UPower daemon as needed.

Depencies
---------
1.  Full ROS Groovy installation - on Ubuntu systems: `$ sudo apt-get install ros-groovy-desktop-full`

Build and usage instructions
----------------------------
First, clone this repository:
```
$ cd /your/catkin/workspace/src
$ git clone https://github.com/calderpg/power_tools.git
$ rospack profile
```
To build all packages in this repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make
```
To build a particular package in the repository:

```
(in the surrounding Catkin workspace directory)
$ catkin_make --pkg <package name>
```
To use, you must source the workspace:

```
(in the surrounding Catkin workspace directory)
$ source devel/setup.bash
```
