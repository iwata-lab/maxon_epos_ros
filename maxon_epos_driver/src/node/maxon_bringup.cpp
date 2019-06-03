/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:49:41
 */

#include <ros/ros.h>
#include <vector>
#include <string>
#include "maxon_epos_driver/EposManager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "maxon_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::vector<std::string> motor_names;
    ros::removeROSArgs(argc, argv, motor_names);
    motor_names.erase(motor_names.begin());

    ros::Rate sleep_rate(50);
    EposManager manager;
    if (!manager.init(nh, pnh, motor_names))
    {
        ROS_FATAL("Failed to initialize EposManager");
        return 1;
    }
    ROS_INFO("Motors Initialized");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        sleep_rate.sleep();
    }

    spinner.stop();

    return 0;
}
