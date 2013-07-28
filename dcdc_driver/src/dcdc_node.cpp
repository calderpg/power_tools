#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "power_msgs/PowerState.h"
extern "C"
{
    #include "dcdc_driver/usbtools.h"
}
extern "C"
{
    #include <usb.h>
}

ros::Publisher g_power_pub;
std::string g_device_id;
double g_update_rate;

power_msgs::PowerState GetDCDCState(struct dcdc_cfg *cfg)
{
    power_msgs::PowerState state;
    state.Manufacturer = "Mini-Box";
    state.DeviceName = "DC-DC";
    state.DeviceID = g_device_id;
    state.DeviceType = state.Converter;
    state.NumCells = 0;
    state.CellType = state.NoType;
    state.NumInputs = 2;
    state.NumOutputs = 1;
    state.SystemTemperature = NAN;
    state.InputID.resize(state.NumInputs);
    state.InputVoltage.resize(state.NumInputs);
    state.InputPower.resize(state.NumInputs, NAN);
    state.OutputID.resize(state.NumOutputs);
    state.OutputVoltage.resize(state.NumOutputs);
    state.OutputPower.resize(state.NumOutputs, NAN);
    state.header.stamp = ros::Time().now();
    uint8_t data[64];
    int error = get_all_values(cfg);
    if (error < 0)
    {
        ROS_ERROR("get_all_values() call to DCDC converter failed with error code: %d", error);
        throw std::string("get_all_values() call to DCDC converter failed");
    }
    error = read_status(cfg, data);
    if (error < 0)
    {
        ROS_ERROR("read_status() call to DCDC converter failed with error code: %d", error);
        throw std::string("read_status() call to DCDC converter failed");
    }
    if (data[0] == DCDCUSB_RECV_ALL_VALUES)
    {
        state.InputVoltage[0] = (double)data[4] * 0.1558f;
        state.InputVoltage[1] = (double)data[3] * 0.1558f;
        state.OutputVoltage[0] = (double)data[5] * 0.1170f;
        ROS_DEBUG("Current state of DCDC: ignition: %f, input: %f, output: %f", state.InputVoltage[0], state.InputVoltage[1], state.OutputVoltage[0]);
        return state;
    }
    else
    {
        ROS_ERROR("Invalid data from DCDC Converter with data[0]: %u", data[0]);
        throw std::string("Invalid data from DCDC converter");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dcdc_converter_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // Load parameters
    nhp.param(std::string("update_rate"), g_update_rate, 1.0);
    nhp.param(std::string("device_id"), g_device_id, std::string("dcdc"));
    g_power_pub = nh.advertise<power_msgs::PowerState>("dcdc_converter/status", 1);
    ROS_INFO("Connecting to the DCDC converter...");
    struct dcdc_cfg cfg;
    int ret = dcdc_init(&cfg, 0);
    while (ros::ok() && ret == DCDC_NO_DEVICE)
    {
        ROS_ERROR("No DCDC Converter found. Retrying in 5 seconds...");
        ros::Duration(5.0).sleep();
        ret = dcdc_init(&cfg, 0);
    }
    if(ret != DCDC_SUCCESS)
    {
        ROS_FATAL("Unable to connect to the DCDC Converter. Exiting...");
        return 1;
    }
    ROS_INFO("Connected to the DCDC converter");
    fflush(stdout);
    ros::Rate spin_rate(g_update_rate);
    while (ros::ok())
    {
        try
        {
            power_msgs::PowerState state = GetDCDCState(&cfg);
            g_power_pub.publish(state);
        }
        catch (...)
        {
            ROS_ERROR("Unable to read state from DCDC converter");
        }
        ros::spinOnce();
        spin_rate.sleep();
    }
    dcdc_stop(&cfg);
    return 0;
}
