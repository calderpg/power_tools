#!/usr/bin/env python

import dbus
import rospy
from power_msgs.msg import *

class DBUS_UPOWER_MONITOR:

    def __init__(self):
        rospy.loginfo("Setting up connection to UPower")
        self.upower_name = "org.freedesktop.UPower"
        self.upower_path = "/org/freedesktop/UPower"
        self.upower_device_name = self.upower_name + ".Device"
        self.system_bus = dbus.SystemBus()
        self.power = self.system_bus.get_object(self.upower_name, self.upower_path)
        self.power_interface = dbus.Interface(self.power, self.upower_name)
        devices = self.power_interface.EnumerateDevices()
        self.power_devices = {}
        self.property_interfaces = {}
        self.publishers = {}
        rospy.loginfo("Setting up connections to individual UPower devices...")
        for device in devices:
            power_device_path = device
            power_device_object = self.system_bus.get_object(self.upower_name, power_device_path)
            power_device = dbus.Interface(power_device_object, self.upower_name)
            self.power_devices[power_device_path] = (power_device)
            property_interface = dbus.Interface(power_device, dbus.PROPERTIES_IFACE)
            self.property_interfaces[power_device_path] = (property_interface)
            publisher_name = rospy.get_namespace() + power_device_path.split("/")[-1]
            self.publishers[power_device_path] = rospy.Publisher(publisher_name, PowerState)
        rospy.loginfo("Connected UPower devices:\n" + str(self.publishers.keys()))
        rospy.loginfo("...Device connections and publishers ready")

    def Loop(self, update_rate):
        rospy.loginfo("Starting to stream UPower state")
        rate = rospy.Rate(update_rate)
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()

    def publish_state(self):
        for device_key in self.publishers.keys():
            device = self.property_interfaces[device_key]
            properties = device.GetAll(self.upower_device_name)
            state = self.process_properties(properties)
            self.publishers[device_key].publish(state)

    def process_properties(self, properties):
        state = PowerState()
        state.header.stamp = rospy.Time().now()
        state.Manufacturer = str(properties["Vendor"])
        state.DeviceName = str(properties["Model"])
        state.DeviceID = str(properties["Serial"])
        state.DeviceType = int(properties["Type"])
        state.TimeToFull = rospy.Duration(float(properties["TimeToFull"]))
        state.TimeToEmpty = rospy.Duration(float(properties["TimeToEmpty"]))
        state.PercentCharge = float(properties["Percentage"])
        state.SystemState = int(properties["State"])
        state.CellType = int(properties["Technology"])
        state.CurrentCharge = float(properties["Energy"])
        state.RatedCapacity = float(properties["EnergyFullDesign"])
        state.CurrentCapacity = float(properties["EnergyFull"])
        state.NumOutputs = 1
        state.OutputID.append("output")
        state.NumInputs = 1
        state.InputID.append("input")
        state.OutputVoltage.append(float(properties["Voltage"]))
        state.OutputPower.append(float(properties["EnergyRate"]))
        return state

if __name__ == "__main__":
    rospy.init_node("upower_monitor")
    update_rate = rospy.get_param("~update_rate", 1.0)
    monitor = DBUS_UPOWER_MONITOR()
    monitor.Loop(update_rate)

