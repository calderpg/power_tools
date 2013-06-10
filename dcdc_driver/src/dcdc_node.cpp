#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "dcdc_driver/Power.h"
extern "C"
{
    #include "dcdc_driver/usbtools.h"
}
extern "C"
{
    #include <usb.h>
}

ros::Publisher g_power_pub;
unsigned int g_seq = 0;

dcdc_driver::Power GetDCDCState(struct dcdc_cfg *cfg)
{
    dcdc_driver::Power state;
    state.ignition = NAN;
    state.input = NAN;
    state.output = NAN;
    state.header.stamp = ros::Time().now();
    state.header.seq = g_seq++;
    uint8_t data[64];
    int error = get_all_values(cfg);
    if (error < 0)
    {
        ROS_ERROR("get_all_values() call failed with error code: %d - Returning NAN", error);
        return state;
    }
    error = read_status(cfg, data);
    if (error < 0)
    {
        ROS_ERROR("read_status() call failed with error code: %d - Returning NAN", error);
        return state;
    }
    if (data[0] == DCDCUSB_RECV_ALL_VALUES)
    {
        state.ignition = (double)data[4] * 0.1558f;
        state.input = (double)data[3] * 0.1558f;
        state.output = (double)data[5] * 0.1170f;
        ROS_DEBUG("Current state of DCDC: ignition: %f, input: %f, output: %f", state.ignition, state.input, state.output);
        return state;
    }
    else
    {
        ROS_ERROR("Invalid data from DCDC Converter with data[0]: %u - Returning NAN", data[0]);
        return state;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dcdc_converter_node");
    ros::NodeHandle nh;
    g_power_pub = nh.advertise<dcdc_driver::Power>("dcdc_converter/status", 1);
    ROS_INFO("Connecting to the DCDC converter...");
    struct dcdc_cfg cfg;
    int ret = dcdc_init(&cfg, 0);
    while (ret == DCDC_NO_DEVICE)
    {
        ROS_ERROR("No DCDC Converter found. Retrying...");
        ret = dcdc_init(&cfg, 0);
    }
    if(ret != DCDC_SUCCESS)
    {
        ROS_FATAL("Unable to connect to the DCDC Converter. Exiting...");
        return 1;
    }
    ROS_INFO("Connected to the DCDC converter");
    fflush(stdout);
    ros::Rate spin_rate(10.0);
    while (ros::ok())
    {
        dcdc_driver::Power state = GetDCDCState(&cfg);
        g_power_pub.publish(state);
        ros::spinOnce();
        spin_rate.sleep();
    }
    dcdc_stop(&cfg);
    return 0;
}
