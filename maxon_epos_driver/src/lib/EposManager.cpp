/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include "maxon_epos_driver/EposManager.hpp"

#include <boost/foreach.hpp>
#include <std_msgs/Float32MultiArray.h>

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

    m_all_position_subscriber = motors_nh.subscribe("all_position", 1000, &EposManager::write, this);
    return true;
}

void EposManager::read()
{
    BOOST_FOREACH (const std::shared_ptr<EposMotor> &motor, m_motors)
    {
        motor->read();
    }
}

void EposManager::write(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < m_motors.size(); i++) {
        std_msgs::Float32 cmd;
        cmd.data = msg->data[i];
        ROS_INFO_STREAM("Send: " << msg->data[i]);
        std_msgs::Float32::ConstPtr msg_cmd(&cmd);
        m_motors[i]->write(msg_cmd);
    }
}
