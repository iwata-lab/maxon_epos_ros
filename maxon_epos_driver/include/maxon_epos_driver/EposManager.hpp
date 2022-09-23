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
#include "EposMotor.hpp"
#include "MotorStates.h"

class EposManager {

public:
    EposManager();
    virtual ~EposManager();

    bool init(rclcpp::NodeHandle &root_nh, rclcpp::NodeHandle &motors_nh,
            const std::vector<std::string> &motor_names);

    void read();

    void write(const maxon_epos_msgs::MotorStates::ConstPtr& msg);

private:
    std::vector<std::shared_ptr<EposMotor>> m_motors;
    rclcpp::Publisher m_all_motor_publisher;
    rclcpp::Subscriber m_all_motor_subscriber;
};

#endif // _EposManager_HPP
