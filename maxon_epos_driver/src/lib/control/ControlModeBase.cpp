/**
 * @file   ControlModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-05 15:28:40
 */

#include "maxon_epos_driver/control/ControlModeBase.hpp"

/**
 * @brief Destructor
 */
ControlModeBase::~ControlModeBase()
{}

void ControlModeBase::init(rclcpp::Node &motor_nh, NodeHandle &node_handle)
{
    nh = std::make_shared<rclcpp::Node>(motor_nh.get_name());
    m_epos_handle = node_handle;
    m_use_ros_unit = nh->declare_parameter<bool>("use_ros_unit", false);
}
