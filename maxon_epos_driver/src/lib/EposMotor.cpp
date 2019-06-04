/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:15:22
 */

#include "maxon_epos_driver/EposMotor.hpp"

/**
 * @brief Constructor
 */
EposMotor::EposMotor()
    : m_position(0), m_velocity(0), m_effort(0), m_current(0) {}

/**
 * @brief Destructor
 */
EposMotor::~EposMotor()
{
}


void EposMotor::init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name)
{
    m_motor_name = motor_name;
}

void EposMotor::read()
{

}

void EposMotor::write()
{

}

void EposMotor::initEposDeviceHandle(ros::NodeHandle &motor_nh)
{

}

void EposMotor::initProtocolStackChanges(ros::NodeHandle &motor_nh)
{

}
