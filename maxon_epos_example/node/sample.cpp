/**
 * @file   sample
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-05 23:51:24
 */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/Float32MultiArray.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh("sample");

    rclcpp::Publisher::SharedPtr pub = nh->create_publisher<std_msgs::msg::Float32MultiArray>("/maxon_bringup/all_position", 1000);
    std_msgs::msg::Float32MultiArray msg = std_msgs::msg::Float32MultiArray();
    msg.data.push_back(300000);
    msg.data.push_back(-200000);
    
    rclcpp::Rate sleep_rate(50);
    while (rclcpp::ok()) {
        pub.publish(msg);
        sleep_rate.sleep();
    }
    
    return 0;
}
