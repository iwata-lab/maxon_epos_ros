/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"


EposProfilePositionMode::~EposProfilePositionMode()
{}

void EposProfilePositionMode::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle)
{
    ControlModeBase::init(motor_nh, node_handle);

    if (m_use_ros_unit) {
        ros::NodeHandle sensor_nh(motor_nh, "encoder");
        int type;
        sensor_nh.param("type", type, 0);

        if (type == 1 || type == 2) {
            int resolution;
            bool inverted_polarity;
            sensor_nh.param("resolution", resolution);
            sensor_nh.param("inverted_poloarity", inverted_polarity);
            m_encoder_resolution = inverted_polarity ? -resolution : resolution;
        } else if (type == 4 || type == 5) {
            int number_of_singleturn_bits;
            bool inverted_polarity;
            sensor_nh.param("number_of_singleturn_bits", number_of_singleturn_bits);
            sensor_nh.param("inverted_poloarity", inverted_polarity);
            m_encoder_resolution = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
        } else {
            throw EposException("Invalid Encoder Type: " + std::to_string(type) + ")");
        }
    }
}

void EposProfilePositionMode::activate()
{
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
}

void EposProfilePositionMode::read()
{}

void EposProfilePositionMode::write(float cmd)
{
    int send_cmd;
    if (m_use_ros_unit) {
        send_cmd = static_cast<int>(cmd * (2. * m_encoder_resolution) / M_PI);
    } else {
        send_cmd = static_cast<int>(cmd);
    }
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, send_cmd, /* absolute */true, /* overwrite */true);
}
