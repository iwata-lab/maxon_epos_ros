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
#include <boost/shared_ptr.hpp>

#include "maxon_epos_driver/EposMotor.hpp"

class EposManager {

public:
    EposManager();
    virtual ~EposManager() throw();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
            const std::vector<std::string> &motor_names);

    void read();

    void write();

private:
    std::vector< boost::shared_ptr< EposMotor >> m_motors;

};

#endif // _EposManager_HPP
