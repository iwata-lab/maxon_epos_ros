/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:42:41
 */

#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"

EposProfileVelocityMode::~EposProfileVelocityMode()
{}

void EposProfileVelocityMode::init(rclcpp::Node &motor_nh, NodeHandle &node_handle)
{
    ControlModeBase::init(motor_nh, node_handle);
}

void EposProfileVelocityMode::activate()
{}

void EposProfileVelocityMode::read()
{}

void EposProfileVelocityMode::write(const double position, const double velocity, const double current)
{}
