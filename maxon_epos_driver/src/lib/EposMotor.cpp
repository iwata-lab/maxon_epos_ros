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

#include "maxon_epos_msgs/MotorState.h"

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
    m_motor_name = motor_name;
    initEposDeviceHandle(motor_nh);

    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);

    initDeviceError();
    initProtocolStackChanges(motor_nh);
    initControlMode(root_nh, motor_nh);
    initEncoderParams(motor_nh);
    initProfilePosition(motor_nh);
    initMiscParams(motor_nh);

    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, m_epos_handle);

    m_state_publisher = motor_nh.advertise<maxon_epos_msgs::MotorState>("get_state", 100);
    m_state_subscriber = motor_nh.subscribe("set_state", 100, &EposMotor::writeCallback, this);
}

maxon_epos_msgs::MotorState EposMotor::read()
{
    try {
        if (m_control_mode) {
            m_control_mode->read();
        }
        m_position = ReadPosition();
        m_velocity = ReadVelocity();
        m_current = ReadCurrent();
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
    maxon_epos_msgs::MotorState msg;
    msg.motor_name = m_motor_name;
    msg.position = m_position;
    msg.velocity = m_velocity;
    msg.current = m_current;
    m_state_publisher.publish(msg);
    return msg;
}

void EposMotor::write(const double position, const double velocity, const double current)
{
    try {
        if (m_control_mode) {
            m_control_mode->write(position, velocity, current);
        }
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}

void EposMotor::writeCallback(const maxon_epos_msgs::MotorState::ConstPtr &msg)
{
    write(msg->position, msg->velocity, msg->current);
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
 * @brief Clear initial errors
 */
void EposMotor::initDeviceError()
{
    unsigned char num_of_device_errors;
    // Get Current Error nums
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    for (int i = 1; i <= num_of_device_errors; i++) {
        unsigned int device_error_code;
        VCS_NODE_COMMAND(GetDeviceErrorCode, m_epos_handle, i, &device_error_code);
        ROS_WARN_STREAM("EPOS Device Error: 0x" << std::hex << device_error_code);
    }

    // Clear Errors
    VCS_NODE_COMMAND_NO_ARGS(ClearFault, m_epos_handle);
    // Get Current Error nums again
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    if (num_of_device_errors > 0) {
        throw EposException(m_motor_name + ": " + std::to_string(num_of_device_errors) + " faults uncleared");
    }
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

    if (type == 1 || type == 2) {
        // Incremental Encoder
        const int resolution(encoder_nh.param("resolution", 0));
        const int gear_ratio(encoder_nh.param("gear_ratio", 0));
        if (resolution == 0 || gear_ratio == 0) {
            throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
        }
        const bool inverted_polarity(encoder_nh.param("inverted_polarity", false));
        VCS_NODE_COMMAND(SetIncEncoderParameter, m_epos_handle, resolution, inverted_polarity);

        m_max_qc = inverted_polarity ? - 4 * resolution * gear_ratio : 4 * resolution * gear_ratio;

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
        m_max_qc = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
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
 * @brief Read Motor Position Function
 *
 * @return motor position
 */
double EposMotor::ReadPosition()
{
    int raw_position;
    double position;
    VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &raw_position);
    if (m_use_ros_unit) {
        // quad-counts of the encoder -> rad
        position = (raw_position / m_max_qc) * 2. * M_PI;
    } else {
        position = raw_position;
    }
    return position;
}

/**
 * @brief Read Motor Velocity Function
 *
 * @return motor velocity
 */
double EposMotor::ReadVelocity()
{
    int raw_velocity;
    double velocity;
    VCS_NODE_COMMAND(GetVelocityIs, m_epos_handle, &raw_velocity);
    if (m_use_ros_unit) {
        // rpm -> rad/s
        velocity = raw_velocity * M_PI / 30.;
    } else {
        velocity = raw_velocity;
    }
    return velocity;
}

/**
 * @brief Read Motor Current Funciton
 *
 * @return motor current
 */
double EposMotor::ReadCurrent()
{
    short raw_current;
    double current;
    VCS_NODE_COMMAND(GetCurrentIs, m_epos_handle, &raw_current);
    if (m_use_ros_unit) {
        // mA -> A
        current = raw_current / 1000.;
    } else {
        current = raw_current;
    }
    return current;
}
