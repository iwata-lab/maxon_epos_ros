/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include "maxon_epos_driver/EposManager.hpp"

#include <boost/foreach.hpp>

/**
 * @brief Constructor
 */
EposManager::EposManager() = default;

/**
 * @brief Destructor
 */
EposManager::~EposManager() = default;


bool EposManager::init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
        const std::vector<std::string> &motor_names)
{
    BOOST_FOREACH (const std::string &motor_name, motor_names)
    {
    }
    return true;
}
