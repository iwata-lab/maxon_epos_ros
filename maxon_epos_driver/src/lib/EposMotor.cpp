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

#include "maxon_epos_msgs/msg/motor_state.hpp"


// TODO check if only declare_parameter is necessary
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
        RCLCPP_ERROR_STREAM(m_control_mode->nh->get_logger(), e.what());
    }
}


void EposMotor::init(rclcpp::Node &root_nh, rclcpp::Node &motor_nh, const std::string &motor_name)
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

    m_state_publisher = motor_nh.create_publisher<maxon_epos_msgs::msg::MotorState>("get_state", 100);
    m_state_subscriber = motor_nh.create_subscription<maxon_epos_msgs::msg::MotorState>("set_state", 100, std::bind(&EposMotor::writeCallback, this, std::placeholders::_1));
}

maxon_epos_msgs::msg::MotorState EposMotor::read()
{
    try {
        if (m_control_mode) {
            m_control_mode->read();
        }
        m_position = ReadPosition();
        m_velocity = ReadVelocity();
        m_current = ReadCurrent();
    } catch (const EposException &e) {
        RCLCPP_ERROR_STREAM(m_control_mode->nh->get_logger(), e.what());
    }
    maxon_epos_msgs::msg::MotorState msg;
    msg.motor_name = m_motor_name;
    msg.position = m_position;
    msg.velocity = m_velocity;
    msg.current = m_current;
    m_state_publisher->publish(msg);
    return msg;
}

void EposMotor::write(const double position, const double velocity, const double current)
{
    try {
        if (m_control_mode) {
            m_control_mode->write(position, velocity, current);
        }
    } catch (const EposException &e) {
        RCLCPP_ERROR_STREAM(m_control_mode->nh->get_logger(), e.what());
    }
}

void EposMotor::writeCallback(const maxon_epos_msgs::msg::MotorState::SharedPtr msg)
{
    write(msg->position, msg->velocity, msg->current);
}

/**
 * @brief Initialize Epos Node Handle
 *
 * @param motor_nh rclcpp Node of motor
 */
void EposMotor::initEposDeviceHandle(rclcpp::Node &motor_nh)
{
    std::string device, protocol_stack, interface, port;
    motor_nh.declare_parameter<std::string>("device", "EPOS4");
    motor_nh.declare_parameter<std::string>("protocol_stack","MAXON SERIAL  V2");
    motor_nh.declare_parameter<std::string>("interface", "USB");
    motor_nh.declare_parameter<std::string>("port", "USB0");
    motor_nh.get_parameter<std::string>("device", device);
    motor_nh.get_parameter<std::string>("protocol_stack", protocol_stack);
    motor_nh.get_parameter<std::string>("interface", interface);
    motor_nh.get_parameter<std::string>("port", port);
    const DeviceInfo device_info(device, protocol_stack, interface, port);
    const unsigned short node_id(motor_nh.declare_parameter<unsigned short >("node_id", 0));
    RCLCPP_WARN(motor_nh.get_logger(), "Node name: %s", motor_nh.get_name());
    RCLCPP_WARN(motor_nh.get_logger(), "Node namespace: %s", motor_nh.get_namespace());
    // motor_nh.declare_parameter<unsigned short >("node_id", 0);
    // unsigned short node_id;
    // motor_nh.get_parameter<unsigned short>("node_id", node_id);



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
        RCLCPP_WARN_STREAM(m_control_mode->nh->get_logger(),"EPOS Device Error: 0x" << std::hex << device_error_code);
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
void EposMotor::initProtocolStackChanges(rclcpp::Node &motor_nh)
{
    // load values from ros parameter server
    const unsigned int baudrate = (uint) motor_nh.declare_parameter<int>("baudrate", 0);
    const unsigned int timeout = (uint) motor_nh.declare_parameter<int>("timeout", 0);
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
void EposMotor::initControlMode(rclcpp::Node &root_nh, rclcpp::Node &motor_nh)
{
    const std::string control_mode(motor_nh.declare_parameter<std::string>("control_mode", "profile_position"));
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
void EposMotor::initEncoderParams(rclcpp::Node &motor_nh)
{
    rclcpp::Node encoder_nh("encoder", motor_nh.get_name());
    
    int type, resolution, gear_ratio;
    encoder_nh.declare_parameter<int>("type",0);
    encoder_nh.get_parameter("type", type);
    VCS_NODE_COMMAND(SetSensorType, m_epos_handle, type);

    if (type == 1 || type == 2) {
        // Incremental Encoder
        encoder_nh.declare_parameter<int>("resolution",0);
        encoder_nh.declare_parameter<int>("gear_ratio",0);
        encoder_nh.get_parameter("resolution", resolution);
        encoder_nh.get_parameter("gear_ratio", gear_ratio);
        if (resolution == 0 || gear_ratio == 0) {
            throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
        }
        bool inverted_polarity;
        encoder_nh.declare_parameter<bool>("inverted_poloarity",false);
        encoder_nh.get_parameter("inverted_polarity", inverted_polarity);
        if (inverted_polarity) {
            RCLCPP_INFO_STREAM(m_control_mode->nh->get_logger(), m_motor_name + ": Inverted polarity is True");
        }
        VCS_NODE_COMMAND(SetIncEncoderParameter, m_epos_handle, resolution, inverted_polarity);

        m_max_qc = 4 * resolution * gear_ratio;

    } else if (type == 4 || type == 5) {
        // SSI Abs Encoder
        bool inverted_polarity = encoder_nh.declare_parameter<bool>("inverted_polarity", false);
        int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
        
        if (encoder_nh.has_parameter("data_rate") && encoder_nh.has_parameter("number_of_singleturn_bits") && encoder_nh.has_parameter("number_of_multiturn_bits")) {
            encoder_nh.get_parameter("data_rate", data_rate);
            encoder_nh.get_parameter("number_of_multiturn_bits", number_of_multiturn_bits);
            encoder_nh.get_parameter("number_of_singleturn_bits", number_of_singleturn_bits);
        } else {
            RCLCPP_ERROR(m_control_mode->nh->get_logger(), "Please set 'data_rate', 'number_of_singleturn_bits', and 'number_of_multiturn_bits'");
        }
        VCS_NODE_COMMAND(SetSsiAbsEncoderParameter, m_epos_handle, data_rate, number_of_multiturn_bits, number_of_singleturn_bits, inverted_polarity);
        m_max_qc = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
    } else {
        // Invalid Encoder
        throw EposException("Invalid Encoder Type: " + std::to_string(type));
    }
}

void EposMotor::initProfilePosition(rclcpp::Node &motor_nh)
{
    rclcpp::Node profile_position_nh("profile_position", motor_nh.get_name());
    if (profile_position_nh.has_parameter("velocity")) {
        int velocity, acceleration, deceleration;
        profile_position_nh.get_parameter("velocity", velocity);
        profile_position_nh.get_parameter("acceleration", acceleration);
        profile_position_nh.get_parameter("deceleration", deceleration);
        VCS_NODE_COMMAND(SetPositionProfile, m_epos_handle, velocity, acceleration, deceleration);
    }
}

/**
 * @brief Initialize other parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initMiscParams(rclcpp::Node &motor_nh)
{
    // use ros unit or default epos unit
    m_use_ros_unit = motor_nh.declare_parameter<bool>("use_ros_unit", false);
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
        position = (raw_position / static_cast<double>(m_max_qc)) * 2. * M_PI;
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
