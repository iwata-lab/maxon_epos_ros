/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz -> revised by cristinaluna
 * @date   2019-05-24 17:49:41
 */

#include <rclcpp/rclcpp.h>
#include <vector>
#include <string>
#include "maxon_epos_driver/EposManager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, "maxon_bringup");
    rclcpp::NodeHandle nh;
    rclcpp::NodeHandle private_nh("~");

    std::vector<std::string> motor_names;
    if (!private_nh.getParam("motor_names", motor_names)) {
        ROS_FATAL("Failed to load motor_names");
        return 1;
    }

    rclcpp::Rate sleep_rate(50);
    EposManager manager;
    if (!manager.init(nh, private_nh, motor_names))
    {
        ROS_FATAL("Failed to initialize EposManager");
        return 1;
    }
    ROS_INFO("Motors Initialized");

    rclcpp::AsyncSpinner spinner(1);
    spinner.start();

    while (rclcpp::ok()) {
        manager.read();
        sleep_rate.sleep();
    }

    spinner.stop();

    return 0;
}
