/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:49:41
 */

#include <ros/ros.h>
#include "maxon_epos_driver/EposCommunication.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "maxon_bringup");
    ros::NodeHandle nh("~");

    EposCommunication ec(nh);
    if (!ec.Connect())
        exit(-1);
    ec.Main();
    ec.CloseDevice();

    return 0;
}
