/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:15:22
 */

#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/EposMotor.hpp"
#include "maxon_epos_driver/utils/Macros.hpp"
#include "maxon_epos_driver/utils/EposException.hpp"
#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"
#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"
#include "maxon_epos_driver/control/EposCurrentMode.hpp"

#include <std_msgs/Float32.h>

/**
 * @brief Constructor
 */
EposMotor::EposMotor()
    : m_position(0), m_velocity(0), m_effort(0), m_current(0)
{}

/**
 * @brief Destructor
 *        change the device state to "disable"
 */
EposMotor::~EposMotor()
{
    try {
        VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}


void EposMotor::init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name)
{
    m_position_publisher = motor_nh.advertise<std_msgs::Float32>("position", 1000);
    m_velocity_publisher = motor_nh.advertise<std_msgs::Float32>("velocity", 1000);
    m_current_publisher = motor_nh.advertise<std_msgs::Float32>("current", 1000);
    m_motor_name = motor_name;
    initEposDeviceHandle(motor_nh);
    initProtocolStackChanges(motor_nh);

    // これいる?
    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);

    initControlMode(root_nh, motor_nh);
    initEncoderParams(motor_nh);
    initProfilePosition(motor_nh);
    initMiscParams(motor_nh);

    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, m_epos_handle);
    m_position_subscriber = motor_nh.subscribe("position_command", 1000, &EposMotor::writeCallback, this);
}

void EposMotor::read()
{
    try {
        if (m_control_mode) {
            m_control_mode->read();
        }
        ReadJointStates();
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
    std_msgs::Float32 position_msg, velocity_msg, current_msg;
    position_msg.data = m_position; velocity_msg.data = m_velocity; current_msg.data = m_current;
    m_position_publisher.publish(position_msg);
    m_velocity_publisher.publish(velocity_msg);
    m_current_publisher.publish(current_msg);
}

void EposMotor::write(const float cmd)
{
    try {
        if (m_control_mode) {
            m_control_mode->write(cmd);
        }
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}

void EposMotor::writeCallback(const std_msgs::Float32::ConstPtr &msg)
{
    write(msg->data);
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
                                 motor_nh.param<std::string>("interface", "USB"),
                                 motor_nh.param<std::string>("port", "USB0"));
    const unsigned short node_id(motor_nh.param("node_id", 0));

    // create epos handle
    m_epos_handle = HandleManager::CreateEposHandle(device_info, node_id);
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

    // 謎 => epos_handleの所有権が1のときっていつ?
    if (m_epos_handle.ptr.use_count() != 1) {
        ROS_WARN_STREAM(motor_nh.getNamespace() << "/{baudrate,timeout} is Ignored. "
                << "Only the first initialized node in a device can set protocol_stack settings.");
        return;
    }

    if (baudrate > 0 && timeout > 0) {
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(), baudrate, timeout);
    } else {
        unsigned int current_baudrate, current_timeout;
        VCS_COMMAND(GetProtocolStackSettings, m_epos_handle.ptr.get(), &current_baudrate, &current_timeout);
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(),
                baudrate > 0 ? baudrate : current_baudrate,
                timeout > 0 ? timeout : current_timeout);
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
    m_control_mode->init(motor_nh, m_epos_handle);
}

/**
 * @brief Initialize EPOS sensor(Encoder) parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initEncoderParams(ros::NodeHandle &motor_nh)
{
    ros::NodeHandle encoder_nh(motor_nh, "encoder");
    
    const int type(encoder_nh.param("type", 0));
    VCS_NODE_COMMAND(SetSensorType, m_epos_handle, type);

    m_encoder_resolution = 0;

    if (type == 1 || type == 2) {
        // Incremental Encoder
        int resolution = encoder_nh.param("resolution", 0);
        bool inverted_polarity = encoder_nh.param("inverted_polarity", false);
        VCS_NODE_COMMAND(SetIncEncoderParameter, m_epos_handle, resolution, inverted_polarity);
        m_encoder_resolution = inverted_polarity ? -resolution : resolution;
    } else if (type == 4 || type == 5) {
        // SSI Abs Encoder
        bool inverted_polarity;
        int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
        encoder_nh.param("inverted_polarity", inverted_polarity, false);
        if (encoder_nh.hasParam("data_rate") && encoder_nh.hasParam("number_of_singleturn_bits") && encoder_nh.hasParam("number_of_multiturn_bits")) {
            encoder_nh.getParam("data_rate", data_rate);
            encoder_nh.getParam("number_of_multiturn_bits", number_of_multiturn_bits);
            encoder_nh.getParam("number_of_singleturn_bits", number_of_singleturn_bits);
        } else {
            ROS_ERROR("Please set 'data_rate', 'number_of_singleturn_bits', and 'number_of_multiturn_bits'");
        }
        VCS_NODE_COMMAND(SetSsiAbsEncoderParameter, m_epos_handle, data_rate, number_of_multiturn_bits, number_of_singleturn_bits, inverted_polarity);
        m_encoder_resolution = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
    } else {
        // Invalid Encoder
        throw EposException("Invalid Encoder Type: " + std::to_string(type));
    }
}

void EposMotor::initProfilePosition(ros::NodeHandle &motor_nh)
{
    ros::NodeHandle profile_position_nh(motor_nh, "profile_position");
    if (profile_position_nh.hasParam("velocity")) {
        int velocity, acceleration, deceleration;
        profile_position_nh.getParam("velocity", velocity);
        profile_position_nh.getParam("acceleration", acceleration);
        profile_position_nh.getParam("deceleration", deceleration);
        VCS_NODE_COMMAND(SetPositionProfile, m_epos_handle, velocity, acceleration, deceleration);
    }
}

/**
 * @brief Initialize other parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initMiscParams(ros::NodeHandle &motor_nh)
{
    // use ros unit or default epos unit
    motor_nh.param("use_ros_unit", m_use_ros_unit, false);
}


/**
 * @brief Helper function for Read
 */
void EposMotor::ReadJointStates()
{
    int raw_position, raw_velocity;
    short raw_current;
    VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &raw_position);
    VCS_NODE_COMMAND(GetVelocityIs, m_epos_handle, &raw_velocity);
    VCS_NODE_COMMAND(GetCurrentIs, m_epos_handle, &raw_current);

    if (m_use_ros_unit) {
        // quad-counts of the encoder -> rad
        m_position = raw_position * M_PI / (2. * m_encoder_resolution);
        // rpm -> rad/s
        m_velocity = raw_velocity * M_PI / 30.;
        // mA -> A
        m_current = raw_current / 1000.;
    } else {
        m_position = raw_position;
        m_velocity = raw_velocity;
        // mA -> A
        m_current = raw_current * 1000.;
    }
}
