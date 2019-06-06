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
#include <std_msgs/Float32.h>
#include "maxon_epos_driver/Device.hpp"
#include "maxon_epos_driver/control/ControlModeBase.hpp"


class EposMotor {
public:
    EposMotor();
    virtual ~EposMotor();

    void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);

    void read();
    void write(const std_msgs::Float32::ConstPtr &msg);

private:
    void initEposDeviceHandle(ros::NodeHandle &motor_nh);
    void initProtocolStackChanges(ros::NodeHandle &motor_nh);
    void initControlMode(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh);
    void initEncoderParams(ros::NodeHandle &motor_nh);
    void initProfilePosition(ros::NodeHandle &motor_nh);
    void initMiscParams(ros::NodeHandle &motor_nh);

    void ReadJointStates();

private:

    typedef std::shared_ptr<ControlModeBase> ControlModePtr;
    typedef std::map<std::string, ControlModePtr> ControlModeMap;

    std::string m_motor_name;

    NodeHandle m_epos_handle;
    ControlModePtr m_control_mode;
    ControlModeMap m_control_mode_map;

    double m_position;
    double m_velocity;
    double m_effort;
    double m_current;

    ros::Publisher m_position_publisher;
    ros::Publisher m_velocity_publisher;
    ros::Publisher m_current_publisher;

    ros::Subscriber m_position_subscriber;

    int m_encoder_resolution;
    bool m_use_ros_unit;
};

#endif // _EposMotor_HPP
