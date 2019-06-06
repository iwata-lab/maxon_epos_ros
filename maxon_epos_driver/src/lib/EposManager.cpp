/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include "maxon_epos_driver/EposManager.hpp"
#include "maxon_epos_msgs/MotorState.h"
#include "maxon_epos_msgs/MotorStates.h"

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
bool EposManager::init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
        const std::vector<std::string> &motor_names)
{
    BOOST_FOREACH (const std::string &motor_name, motor_names)
    {
        ROS_INFO_STREAM("Loading Epos: " << motor_name);
        // Copy constructor => ns = motors_nh's namespace + / + motor_name
        ros::NodeHandle motor_nh(motors_nh, motor_name);

        std::shared_ptr<EposMotor> motor(new EposMotor());
        motor->init(root_nh, motor_nh, motor_name);
        m_motors.push_back(motor);
    }

    m_all_motor_publisher = motors_nh.advertise<maxon_epos_msgs::MotorStates>("get_all_states", 100);
    m_all_motor_subscriber = motors_nh.subscribe("set_all_states", 100, &EposManager::write, this);
    return true;
}

void EposManager::read()
{
    maxon_epos_msgs::MotorStates msg;
    BOOST_FOREACH (const std::shared_ptr<EposMotor> &motor, m_motors)
    {
        msg.states.push_back(motor->read());
    }
    m_all_motor_publisher.publish(msg);
}

void EposManager::write(const maxon_epos_msgs::MotorStates::ConstPtr& msg)
{
    for (int i = 0; i < m_motors.size(); i++) {
        maxon_epos_msgs::MotorState state = msg->states[i];
        ROS_INFO_STREAM("Send: " << state.position);
        m_motors[i]->write(state.position, state.velocity, state.current);
    }
}
