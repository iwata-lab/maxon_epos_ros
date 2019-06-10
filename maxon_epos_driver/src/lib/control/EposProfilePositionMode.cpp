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
        ros::NodeHandle encoder_nh(motor_nh, "encoder");
        const int type(encoder_nh.param("type", 0));

        if (type == 1 || type == 2) {
            const int resolution(encoder_nh.param("resolution", 0));
            const int gear_ratio(encoder_nh.param("gear_ratio", 0));
            if (resolution == 0 || gear_ratio == 0) {
                throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
            }
            const bool inverted_polarity(encoder_nh.param("inverted_poloarity", false));
            m_max_qc = 4 * resolution * gear_ratio;
        } else if (type == 4 || type == 5) {
            int number_of_singleturn_bits;
            bool inverted_polarity;
            encoder_nh.param("number_of_singleturn_bits", number_of_singleturn_bits);
            encoder_nh.param("inverted_poloarity", inverted_polarity);
            m_max_qc = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
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

void EposProfilePositionMode::write(const double position, const double velocity, const double current)
{
    int quad_count;
    ROS_DEBUG_STREAM("Target Position: " << position);
    ROS_DEBUG_STREAM("Encoder Resolution: " << m_max_qc);
    if (m_use_ros_unit) {
        quad_count = static_cast<int>((position / (2 * M_PI)) * m_max_qc);
    } else {
        quad_count = static_cast<int>(position);
    }
    ROS_DEBUG_STREAM("Send Quad Count: " << quad_count);
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);
}
