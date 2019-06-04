/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:07:38
 */

#ifndef _EposProfileVelocityMode_HPP
#define _EposProfileVelocityMode_HPP

#include "maxon_epos_driver/control/ModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposProfileVelocityMode : public ModeBase {
public:
    virtual ~EposProfileVelocityMode();

    virtual void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name, NodeHandle &node_handle);
    virtual void activate();
    virtual void read();
    virtual void write();

private:

};

#endif // _EposProfileVelocityMode_HPP
