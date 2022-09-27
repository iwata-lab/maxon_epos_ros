/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include "maxon_epos_driver/EposManager.hpp"
#include "maxon_epos_msgs/msg/motor_state.h"
#include "maxon_epos_msgs/msg/motor_states.h"

#include <boost/foreach.hpp>

/**
 * @brief Constructor
 */
EposManager::EposManager() = default;

/**
 * @brief Destructor
 */
EposManager::~EposManager() = default;


/**
 * @brief Initialize function
 *
 * @param root_nh 
 * @param motors_nh
 * @param motor_names
 *
 * @return 
 */
bool EposManager::init(rclcpp::Node &root_nh, rclcpp::Node &motors_nh,
        const std::vector<std::string> &motor_names)
{
    BOOST_FOREACH (const std::string &motor_name, motor_names)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Loading Epos: " << motor_name);
        // Copy constructor => ns = motors_nh's namespace + / + motor_name
        rclcpp::Node motor_nh(motor_name, motors_nh.get_name());

        std::shared_ptr<EposMotor> motor(new EposMotor());
        motor->init(root_nh, motor_nh, motor_name);
        m_motors.push_back(motor);
    }

    m_all_motor_publisher = motors_nh.create_publisher<maxon_epos_msgs::msg::MotorStates>("get_all_states", 100);
    m_all_motor_subscriber = motors_nh.create_subscription<maxon_epos_msgs::msg::MotorStates>("set_all_states", 100, std::bind(&EposManager::write, this, std::placeholders::_1));
    return true;
}

void EposManager::read()
{
    maxon_epos_msgs::msg::MotorStates msg;
    BOOST_FOREACH (const std::shared_ptr<EposMotor> &motor, m_motors)
    {
        msg.states.push_back(motor->read());
    }
    m_all_motor_publisher->publish(msg);
}

void EposManager::write(const maxon_epos_msgs::msg::MotorStates::SharedPtr msg)
{
    for (uint i = 0; i < m_motors.size(); i++) {
        maxon_epos_msgs::msg::MotorState state = msg->states[i];
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Send: " << state.position);
        m_motors[i]->write(state.position, state.velocity, state.current);
    }
}
