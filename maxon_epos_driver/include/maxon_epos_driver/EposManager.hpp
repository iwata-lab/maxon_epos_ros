/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:08:23
 */

#ifndef _EposManager_HPP
#define _EposManager_HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "maxon_epos_driver/EposMotor.hpp"

class EposManager {

public:
    EposManager();
    virtual ~EposManager();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
            const std::vector<std::string> &motor_names);

    void read();

    void write(const std_msgs::Float32MultiArray::ConstPtr& msg);

private:
    std::vector<std::shared_ptr<EposMotor>> m_motors;
    ros::Subscriber m_all_position_subscriber;
    ros::Subscriber m_all_velocity_subscriber;
    ros::Subscriber m_all_current_subscriber;
};

#endif // _EposManager_HPP
