/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:04:39
 */

#ifndef _EposProfilePositionMode_HPP
#define _EposProfilePositionMode_HPP

#include "maxon_epos_driver/control/ModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposProfilePositionMode : public ModeBase {
public:
    virtual ~EposProfilePositionMode();

    virtual void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name, NodeHandle &node_handle);
    virtual void activate();
    virtual void read();
    virtual void write();

private:

};

#endif // _EposProfilePositionMode_HPP
