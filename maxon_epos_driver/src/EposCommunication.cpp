/**
 * @file   EposCommunication
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:40:20
 */

#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/EposCommunication.hpp"
#include <ros/ros.h>
#include <cmath>

bool EposCommunication::Test()
{
    // VCS_OpenSubDevice();
    // VCS_GetDeviceNameSelection()
    return true;
}

EposCommunication::EposCommunication(ros::NodeHandle nh)
{
    SetDefaultParameters();
    m_nh = nh;
    m_position_pub = m_nh.advertise<std_msgs::Float32>("position_status", 1000);
    m_position_sub = m_nh.subscribe("position_goal", 1000, &EposCommunication::PositionCallback, this);
}

EposCommunication::~EposCommunication()
{}

/**
 * @brief Connect to EPOS Device
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::Connect()
{
    if (!OpenDevice())
    {
        return false;
    }
    return SetProtocolStack();
}

/**
 * @brief DisConnect to EPOS Device
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::DisConnect()
{
    bool result = CloseDevice();
    return result;
}


/**
 * @brief Change Protocol Stack for example baudrate
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::SetProtocolStack()
{
    int result = MMC_FAILED;
    unsigned int error_code = 0;
    unsigned int timeout = 0;
    unsigned int current_baudrate = 0;

    // Set Protocol
    if(VCS_GetProtocolStackSettings(m_key_handle, &current_baudrate, &timeout, &error_code) != 0)
    {
        VCS_SetProtocolStackSettings(m_key_handle, m_baudrate, timeout, &error_code);
    }

    // Check if protocol value changed
    if(VCS_GetProtocolStackSettings(m_key_handle, &current_baudrate, &timeout, &error_code) != 0)
    {
        if (current_baudrate == m_baudrate)
        {
            LogInfo("Succeeded to Connect Device!");
            result = MMC_SUCCESS;
        }
    }

    if (result == MMC_FAILED)
    {
        LogError("VCS_SetProtocolStackSettings", result, error_code);
        return false;
    }
    return true;
}


/**
 * @brief Connect to Device
 *
 * @return True if succeeded to open, otherwise False
 */
bool EposCommunication::OpenDevice()
{
    int result = MMC_FAILED;

    LogInfo("Open Device .....");

    // Convert from string => char*
    char* device_name         = new char[255];
    char* protocol_stack_name = new char[255];
    char* interface_name      = new char[255];
    char* port_name           = new char[255];
    strcpy(device_name, m_device_name.c_str());
    strcpy(protocol_stack_name, m_protocol_stack_name.c_str());
    strcpy(interface_name, m_interface_name.c_str());
    strcpy(port_name, m_port_name.c_str());

    // Open Device
    unsigned int error_code = 0;
    m_key_handle = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code);

    // Release objects
    delete []device_name;
    delete []protocol_stack_name;
    delete []interface_name;
    delete []port_name;

    if (m_key_handle == 0 && error_code != 0)
    {
        LogFatal("VCS_OpenDevice", result, error_code);
        return false;
    }

    return true;
}

/**
 * @brief Set Default Connecting Paramters
 *      => TODO: Reading from args or params
 */
void EposCommunication::SetDefaultParameters()
{
    m_device_name         = "EPOS2";
    m_protocol_stack_name = "MAXON SERIAL V2";
    m_interface_name      = "USB";
    m_port_name           = "USB0";
    m_baudrate            = 1000000;

    g_usNodeId = 1;
}


/**
 * @brief Main function
 */
void EposCommunication::Main()
{
    PrintSettings();
    PrepareMove();
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        GetAndPublishPosition();
        ros::spinOnce();
        loop_rate.sleep();
    }
    PostMove();
}

/**
 * @brief Print default settings
 */
void EposCommunication::PrintSettings()
{
    std::stringstream msg;
    msg << "===================================================" << std::endl;
    msg << "Default Settings:" << std::endl;
    msg << "node id             = " << g_usNodeId << std::endl;
    msg << "device name         = '" << m_device_name << "'" << std::endl;
    msg << "protocal stack name = '" << m_protocol_stack_name << "'" << std::endl;
    msg << "interface name      = '" << m_interface_name << "'" << std::endl;
    msg << "port name           = '" << m_port_name << "'"<< std::endl;
    msg << "baudrate            = " << m_baudrate << std::endl;
    msg << "===================================================";
    std::cout << msg.str() << std::endl;
}

/**
 * @brief Close Epos Device
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::CloseDevice()
{
    int result = MMC_FAILED;

    unsigned int error_code = 0;

    LogInfo("Close Device......");

    if(VCS_CloseDevice(m_key_handle, &error_code)!=0 && error_code == 0)
    {
        result = MMC_SUCCESS;
    }

    if (result == MMC_FAILED)
    {
        LogError("VCS_CloseDevice", result, error_code);
    }
    return result;
}


/**
 * @brief Prepare for position move
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::PrepareMove()
{
    int result = MMC_SUCCESS;
    int oIsFault = 0;
    unsigned int error_code = 0;

    if(VCS_GetFaultState(m_key_handle, g_usNodeId, &oIsFault, &error_code ) == 0)
    {
        ROS_ERROR("Failed to VCS_GetFaultState (result=%d, error_code=0x%x)", result, error_code);
        result = MMC_FAILED;
    }

    if(result==0)
    {
        if(oIsFault)
        {
            std::stringstream msg;
            msg << "clear fault, node = '" << g_usNodeId << "'";
            ROS_ERROR("Clear Fault, node = '%u'", g_usNodeId);

            if(VCS_ClearFault(m_key_handle, g_usNodeId, &error_code) == 0)
            {
                ROS_ERROR("Failed to VCS_ClearFault (result=%d, error_code=0x%x)", result, error_code);
                result = MMC_FAILED;
            }
        }

        if(result==0)
        {
            int oIsEnabled = 0;

            if(VCS_GetEnableState(m_key_handle, g_usNodeId, &oIsEnabled, &error_code) == 0)
            {
                ROS_ERROR("Failed to VCS_GetEnableState (result=%d, error_code=0x%x)", result, error_code);
                result = MMC_FAILED;
            }

            if(result==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(m_key_handle, g_usNodeId, &error_code) == 0)
                    {
                        ROS_ERROR("Failed to VCS_SetEnableState (result=%d, error_code=0x%x)", result, error_code);
                        result = MMC_FAILED;
                    }
                }
            }
        }
    }

    ROS_INFO("Set profile position mode, node = %u", g_usNodeId);
    if(VCS_ActivateProfilePositionMode(m_key_handle, g_usNodeId, &error_code) == 0)
    {
        ROS_ERROR("Failed to VCS_ActivateProfilePositionMode (result=%d, error_code=0x%x)", result, error_code);
        result = MMC_FAILED;
    }
    return result;
}

/**
 * @brief Postprocess after move
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::PostMove()
{
    int result = MMC_SUCCESS;
    unsigned int error_code = 0;
    if(VCS_SetDisableState(m_key_handle, g_usNodeId, &error_code) == 0)
    {
        LogError("VCS_SetDisableState", result, error_code);
        result= MMC_FAILED;
    }
    return result;
}

/**
 * @brief Position Move
 *
 * @param target_position Position goal
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::PositionMove(long target_position)
{
    int result = MMC_SUCCESS;
    unsigned int error_code = 0;

    ROS_INFO("Move to positoin = %ld, node = %u", target_position, g_usNodeId);
    if(VCS_MoveToPosition(m_key_handle, g_usNodeId, target_position, 0, 1, &error_code) == 0)
    {
        LogError("VCS_MoveToPosition", result,  error_code);
        result = MMC_FAILED;
    }

    if(result == MMC_SUCCESS)
    {
        ROS_INFO("Halt position movement");

        if(VCS_HaltPositionMovement(m_key_handle, g_usNodeId, &error_code) == 0)
        {
            LogError("VCS_HaltPositionMovement", result, error_code);
            result = MMC_FAILED;
        }
    }

    return result;
}

/**
 * @brief Get and publish current position in radian
 *
 * @return True if succeeded, otherwise False
 */
bool EposCommunication::GetAndPublishPosition()
{
    unsigned int error_code = 0;
    int current_position = 0;
    int result = MMC_SUCCESS;

    if (VCS_GetPositionIs(m_key_handle, g_usNodeId, &current_position, &error_code) == 0)
    {
        result = MMC_FAILED;
        LogError("VCS_GetPositionIs", result,  error_code);
    }
    else
    {
        result = MMC_SUCCESS;
        std_msgs::Float32 msg;
        msg.data = Convert2Radian(current_position);
        m_position_pub.publish(msg);
    }
    return result;
}

/**
 * @brief Callback method for position_goal topic
 *
 * @param msg std_msgs::Float32 position message
 */
void EposCommunication::PositionCallback(const std_msgs::Float32::ConstPtr& msg)
{
    PositionMove(Convert2Qc(msg->data));
}

/**
 * @brief Convert from encoder value to radian
 *
 * @param position encoder value
 *
 * @return radian value
 */
float EposCommunication::Convert2Radian(long position)
{
    float radian;
    const int ENCODER = 4096;
    const int EPOS2_GAER_DECELERATION_RATIO = 231;
    const int EPOS4_GAER_DECELERATION_RATIO = 319;
    ;

    if (m_device_name == "EPOS2")
    {
        radian = 360 * position / (ENCODER * EPOS2_GAER_DECELERATION_RATIO) / (M_PI / 180);
    }
    else
    {
        radian = 360 * position / (ENCODER * EPOS4_GAER_DECELERATION_RATIO) / (M_PI / 180);
    }

    return radian;
}

/**
 * @brief Convert from radian to encoder value
 *
 * @param radian radian
 *
 * @return encoder value
 */
long EposCommunication::Convert2Qc(float radian)
{
    double motorQc = 0;
    const int ENCODER = 4096;
    const int EPOS2_GAER_DECELERATION_RATIO = 231;
    const int EPOS4_GAER_DECELERATION_RATIO = 319;

    if (m_device_name == "EPOS2")
    {
        motorQc = (radian * 180 / M_PI) * (ENCODER * EPOS2_GAER_DECELERATION_RATIO) / 360;
    }
    else
    {
        motorQc = (radian * 180 / M_PI) * (ENCODER * EPOS4_GAER_DECELERATION_RATIO) / 360;
    }
    long targetPos = std::round(motorQc);

    return targetPos;
}

void EposCommunication::LogInfo(std::string msg)
{
    // ROS_INFO(msg);
}

void EposCommunication::LogError(std::string func, int result, unsigned int error_code)
{
    // ROS_ERROR_STREAM("Failed to " << func << " (result = " << result << ", error_code = 0x" << error_code <<")");
}

void EposCommunication::LogFatal(std::string func, int result, unsigned int error_code)
{
    // ROS_FATAL_STREAM("Failed to " << func << " (result = " << result << ", error_code = 0x" << error_code <<")");
}
