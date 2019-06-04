/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:15:22
 */

#include "maxon_epos_driver/EposMotor.hpp"
#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/utils/EposException.hpp"
#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"
#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"
#include "maxon_epos_driver/control/EposCurrentMode.hpp"

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
    initEposDeviceHandle(motor_nh);
    initProtocolStackChanges(motor_nh);
    initControlMode(root_nh, motor_nh);
}

void EposMotor::read()
{

}

void EposMotor::write()
{

}

/**
 * @brief Initialize Epos Node Handle
 *
 * @param motor_nh ros NodeHandle of motor
 */
void EposMotor::initEposDeviceHandle(ros::NodeHandle &motor_nh)
{
    const DeviceInfo device_info(motor_nh.param<std::string>("device", "EPOS4"),
                                 motor_nh.param<std::string>("protocol_stack", "MAXON SERIAL V2"),
                                 motor_nh.param<std::string>("interface_name", "USB"),
                                 motor_nh.param<std::string>("port_name", "USB0"));
    const unsigned short node_id(motor_nh.param("node_id", 0));

    // create epos handle
    m_epos_handle = CreateEposHandle(device_info, node_id);
}

/**
 * @brief Set Protocol Stack for Epos Node Handle
 *
 * @param motor_nh ros NodeHandle of motor
 */
void EposMotor::initProtocolStackChanges(ros::NodeHandle &motor_nh)
{
    // load values from ros parameter server
    const unsigned int baudrate(motor_nh.param("baudrate", 0));
    const unsigned int timeout(motor_nh.param("timeout", 0));
    if (baudrate == 0 && timeout == 0) {
        return;
    }

    if (m_epos_handle.ptr.use_count() != 1) {
        ROS_WARN_STREAM(motor_nh.getNamespace() << "/{baudrate,timeout} is Ignored. "
                << "Only the first initialized node in a device can set protocol_stack settings.");
        return;
    }

    unsigned int error_code;
    // TODO: Create Macro

    if (baudrate > 0 && timeout > 0) {
        VCS_SetProtocolStackSettings(m_epos_handle.ptr.get(), baudrate, timeout, &error_code);
    } else {
        unsigned int current_baudrate, current_timeout;
        VCS_GetProtocolStackSettings(m_epos_handle.ptr.get(), &current_baudrate, &current_timeout, &error_code);
        VCS_SetProtocolStackSettings(m_epos_handle.ptr.get(),
                baudrate > 0 ? baudrate : current_baudrate,
                timeout > 0 ? timeout : current_timeout,
                &error_code);
    }
}

/**
 * @brief Initialize Control Mode
 *
 * @param root_nh NodeHandle of TopLevel
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initControlMode(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh)
{
    const std::string control_mode(motor_nh.param<std::string>("control_mode", "profile_position"));
    if (control_mode == "profile_position") {
        m_control_mode.reset(new EposProfilePositionMode());
    } else if (control_mode == "profile_velocity") {
        m_control_mode.reset(new EposProfileVelocityMode());
    } else if (control_mode == "current") {
        m_control_mode.reset(new EposCurrentMode());
    } else {
        throw EposException("Unsupported control mode (" + control_mode + ")");
    }
    m_control_mode->init(root_nh, motor_nh, m_motor_name, m_epos_handle);
}
