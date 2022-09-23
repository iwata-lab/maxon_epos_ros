/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:04:39
 */

#ifndef _EposProfilePositionMode_HPP
#define _EposProfilePositionMode_HPP

#include "ControlModeBase.hpp"
#include "../Device.hpp"
#include <rclcpp/rclcpp.hpp>


class EposProfilePositionMode : public ControlModeBase {
public:
    virtual ~EposProfilePositionMode();

    virtual void init(rclcpp::NodeHandle &motor_nh, NodeHandle &node_handle);
    virtual void activate();
    virtual void read();
    virtual void write(const double position, const double velocity, const double current);

private:
    int m_max_qc;
    double m_position_cmd;
};

#endif // _EposProfilePositionMode_HPP
