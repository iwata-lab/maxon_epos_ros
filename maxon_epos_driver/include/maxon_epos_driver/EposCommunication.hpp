/**
 * @file   EposCommunication
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:37:54
 */

#ifndef _EposCommunication_HPP
#define _EposCommunication_HPP

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#include <string>
#include <ros/ros.h>
#include "Definitions.h"
#include <std_msgs/Float32.h>

class EposCommunication {
public:

    EposCommunication(ros::NodeHandle nh);
    virtual ~EposCommunication();

    void Main();
    bool OpenDevice();
    bool CloseDevice();
    bool Test();

    bool Connect();
    bool DisConnect();

    bool SetProtocolStack();

private:
    bool PrepareMove();
    bool PostMove();
    /**
     * @brief Print default settings
     */
    void PrintSettings();
    void SetDefaultParameters();
    bool PositionMove(long target_position);

    /**
     * @brief Subscriber callback
     *        Write position to motor
     *
     * @param msg Float32 msg
     */
    void PositionCallback(const std_msgs::Float32::ConstPtr& msg);

    /**
     * @brief Get and publish current motor position
     *
     * @return 
     */
    bool GetAndPublishPosition();

    void LogInfo(std::string msg);
    void LogError(std::string func, int result, unsigned int error_code);
    void LogFatal(std::string func, int result, unsigned int error_code);

    unsigned short g_usNodeId = 1;

    /**
     * @brief Convert from encoder value to radian
     *
     * @param current encoder value
     *
     * @return radian
     */
    float Convert2Radian(long position);
    /**
     * @brief Convert from radian to encoder value
     *
     * @param radian radian
     *
     * @return encoder value
     */
    long Convert2Qc(float radian);


    /**
     * @brief Handle for communication port access
     */
    void* m_key_handle;

    /**
     * @brief Name of connected device:
     *      1. EPOS2
     *      2. EPOS4
     *      3. EPOS
     */
    std::string m_device_name;
    /**
     * @brief Name of used communication protocol:
     *      1. MAXON_RSS232
     *      2. MAXON SERIAL V2
     *      3. CANopen
     */
    std::string m_protocol_stack_name;

    /**
     * @brief Name of Interface:
     *      1. RS232
     *      2. USB
     */
    std::string m_interface_name;

    /**
     * @brief Name of port:
     *      1. COM1, COM2, ....
     *      2. USB0, USB1, ....
     *      3. CAN0, CAN1, ....
     */
    std::string m_port_name;

    /**
     * @brief Baudrate
     */
    int m_baudrate;

    /**
     * @brief ROS Node Handle
     */
    ros::NodeHandle m_nh;

    /**
     * @brief Current Position Publisher
     */
    ros::Publisher m_position_pub;

    /**
     * @brief Position Subscriber
     */
    ros::Subscriber m_position_sub;
};

#endif // _EposCommunication_HPP
