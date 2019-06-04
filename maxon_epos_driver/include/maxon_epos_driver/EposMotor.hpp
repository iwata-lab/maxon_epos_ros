/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:12:59
 */

#ifndef _EposMotor_HPP
#define _EposMotor_HPP

#include <string>
#include <ros/ros.h>
#include "maxon_epos_driver/Device.hpp"


class EposMotor {
public:
    EposMotor();
    virtual ~EposMotor();

    void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);

    void read();
    void write();

private:
    void initEposDeviceHandle(ros::NodeHandle &motor_nh);
    void initProtocolStackChanges(ros::NodeHandle &motor_nh);

private:
    std::string m_motor_name;

    double m_position;
    double m_velocity;
    double m_effort;
    double m_current;
};

#endif // _EposMotor_HPP
