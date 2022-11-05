/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"


EposProfilePositionMode::~EposProfilePositionMode()
{}

void EposProfilePositionMode::init(rclcpp::Node &motor_nh, NodeHandle &node_handle)
{
    ControlModeBase::init(motor_nh, node_handle);

    if (m_use_ros_unit) {
        rclcpp::Node encoder_nh("encoder", motor_nh.get_name());
        encoder_nh.declare_parameter<int>("type",0);
        int type, resolution, gear_ratio;
        encoder_nh.get_parameter("type", type);
        // const int type(encoder_nh.get_parameter("type", 0));

        if (type == 1 || type == 2) {
            // const int resolution(encoder_nh.get_parameter("resolution", 0));
            // const int gear_ratio(encoder_nh.get_parameter("gear_ratio", 0));
            encoder_nh.declare_parameter<int>("resolution",0);
            encoder_nh.declare_parameter<int>("gear_ratio",0);
            encoder_nh.get_parameter("resolution", resolution);
            encoder_nh.get_parameter("gear_ratio", gear_ratio);
            if (resolution == 0 || gear_ratio == 0) {
                throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
            }
            // const bool inverted_polarity(encoder_nh.get_parameter("inverted_poloarity", false));
            bool inverted_polarity;
            encoder_nh.declare_parameter<bool>("inverted_poloarity",false);
            encoder_nh.get_parameter("inverted_polarity", inverted_polarity);
            m_max_qc = 4 * resolution * gear_ratio;
        } else if (type == 4 || type == 5) {
            int number_of_singleturn_bits;
            bool inverted_polarity;
            // encoder_nh.get_parameter("number_of_singleturn_bits", number_of_singleturn_bits);
            // encoder_nh.get_parameter("inverted_poloarity", inverted_polarity);
            encoder_nh.declare_parameter<int>("number_of_singleturn_bits",0);
            encoder_nh.declare_parameter<bool>("inverted_poloarity",false);
            encoder_nh.get_parameter("number_of_singleturn_bits", number_of_singleturn_bits);
            encoder_nh.get_parameter("inverted_poloarity", inverted_polarity);
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
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("encoder"), "Target Position: " << position);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("encoder"), "Encoder Resolution: " << m_max_qc);
    if (m_use_ros_unit) {
        quad_count = static_cast<int>((position / (2 * M_PI)) * m_max_qc);
    } else {
        quad_count = static_cast<int>(position);
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("encoder"), "Send Quad Count: " << quad_count);
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);
}
