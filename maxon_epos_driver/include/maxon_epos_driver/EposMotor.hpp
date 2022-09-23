/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:12:59
 */

#ifndef _EposMotor_HPP
#define _EposMotor_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "Device.hpp"
#include "control/ControlModeBase.hpp"
#include "maxon_epos_msgs/MotorState.h"


class EposMotor {
public:
    EposMotor();
    virtual ~EposMotor();

    void init(rclcpp::NodeHandle &root_nh, rclcpp::NodeHandle &motor_nh,
            const std::string &motor_name);

    maxon_epos_msgs::MotorState read();
    void write(const double position, const double velocity, const double current);

private:
    void initEposDeviceHandle(rclcpp::NodeHandle &motor_nh);
    void initDeviceError();
    void initProtocolStackChanges(rclcpp::NodeHandle &motor_nh);
    void initControlMode(rclcpp::NodeHandle &root_nh, rclcpp::NodeHandle &motor_nh);
    void initEncoderParams(rclcpp::NodeHandle &motor_nh);
    void initProfilePosition(rclcpp::NodeHandle &motor_nh);
    void initMiscParams(rclcpp::NodeHandle &motor_nh);

    double ReadPosition();
    double ReadVelocity();
    double ReadCurrent();

    void writeCallback(const maxon_epos_msgs::MotorState::ConstPtr &msg);

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

    rclcpp::Publisher m_state_publisher;
    rclcpp::Subscriber m_state_subscriber;

    int m_max_qc;
    bool m_use_ros_unit;
};

#endif // _EposMotor_HPP
