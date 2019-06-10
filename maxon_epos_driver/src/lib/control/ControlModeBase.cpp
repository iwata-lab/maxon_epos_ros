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

void ControlModeBase::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle)
{
    m_epos_handle = node_handle;
    m_use_ros_unit = motor_nh.param("use_ros_unit", false);
}
