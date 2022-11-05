/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:08:23
 */

#ifndef _EposManager_HPP
#define _EposManager_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "maxon_epos_driver/EposMotor.hpp"
#include "maxon_epos_msgs/msg/motor_states.hpp"

class EposManager {

public:
    EposManager();
    virtual ~EposManager();

    bool init(rclcpp::Node &root_nh, rclcpp::Node &motors_nh,
            const std::vector<std::string> &motor_names);

    void read();

    void write(const maxon_epos_msgs::msg::MotorStates::SharedPtr msg);

private:
    std::vector<std::shared_ptr<EposMotor>> m_motors;
    rclcpp::Publisher<maxon_epos_msgs::msg::MotorStates>::SharedPtr m_all_motor_publisher;
    rclcpp::Subscription<maxon_epos_msgs::msg::MotorStates>::SharedPtr m_all_motor_subscriber;
};

#endif // _EposManager_HPP
