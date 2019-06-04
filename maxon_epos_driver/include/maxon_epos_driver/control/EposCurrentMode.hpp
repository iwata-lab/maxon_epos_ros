/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:08:10
 */

#ifndef _EposCurrentMode_HPP
#define _EposCurrentMode_HPP

#include "maxon_epos_driver/control/ModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposCurrentMode : public ModeBase {
public:
    virtual ~EposCurrentMode();

    virtual void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name, NodeHandle &node_handle);
    virtual void activate();
    virtual void read();
    virtual void write();

private:

};

#endif // _EposCurrentMode_HPP
