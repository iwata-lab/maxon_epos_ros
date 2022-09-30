/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz -> revised by cristinaluna
 * @date   2019-05-24 17:49:41
 */

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include "maxon_epos_driver/EposManager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("maxon_bringup");
    rclcpp::Node private_nh("maxon_bringup_private");
    nh->declare_parameter<std::vector<std::string>>("motor_names", {});

    std::vector<std::string> motor_names;
    nh->get_parameter<std::vector<std::string>>("motor_names", motor_names);
    if (motor_names.size() <= 0) {
        RCLCPP_FATAL(nh->get_logger(), "Failed to load motor_names");
        return 1;
    }

    rclcpp::Rate sleep_rate(50);
    EposManager manager;
    if (!manager.init(*nh, private_nh, motor_names))
    {
        RCLCPP_FATAL(nh->get_logger(), "Failed to initialize EposManager");
        return 1;
    }
    RCLCPP_INFO(nh->get_logger(), "Motors Initialized");

    // rclcpp::AsyncSpinner spinner(1);
    // spinner.start();

    while (rclcpp::ok()) {
        manager.read();
        sleep_rate.sleep();
    }

    // spinner.stop();

    return 0;
}
