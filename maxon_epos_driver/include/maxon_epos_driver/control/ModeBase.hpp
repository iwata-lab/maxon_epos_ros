/**
 * @file   ModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:31:01
 */

#ifndef _ModeBase_HPP
#define _ModeBase_HPP

#include <string>
#include <vector>
#include <ros/ros.h>

class ModeBase {
public:
    ModeBase() {}
    virtual ~ModeBase() {}

    virtual void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name);

    // activate operation mode
    virtual void activate() = 0;

    // read something required for operation mode
    virtual void read() = 0;

    // write commands of operation mode
    virtual void write() = 0;
};

#endif // _ModeBase_HPP
