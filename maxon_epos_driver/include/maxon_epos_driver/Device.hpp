/**
 * @file   Device
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 12:13:53
 */

#ifndef _Device_HPP
#define _Device_HPP

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

/**
 * @brief Information of device
 */
class DeviceInfo {
public:
    DeviceInfo();
    DeviceInfo(const std::string &device_name, const std::string &protocol_stack,
               const std::string &interface_name, const std::string &portname);
    virtual ~DeviceInfo();

public:
    std::string m_device_name;
    std::string m_protocol_stack;
    std::string m_interface_name;
    std::string m_port_name;
};


/**
 * @brief Handle of device
 */
class DeviceHandle {
public:
    DeviceHandle();
    DeviceHandle(const DeviceInfo& info);
    virtual ~DeviceHandle();

private:
    static boost::shared_ptr<void> MakePtr(const DeviceInfo &device_info);
    static void* OpenDevice(const DeviceInfo &device_info);
    static void CloseDevice(void *raw_device_ptr);

public:
    boost::shared_ptr<void> ptr;
};


/**
 * @brief Information of Node
 */
class NodeInfo : public DeviceInfo {
public:
    NodeInfo();
    NodeInfo(const DeviceInfo &device_info, const unsigned short node_id);
    virtual ~NodeInfo();

public:
    unsigned short m_node_id;
};


/**
 * @brief Handle of node
 */
class NodeHandle : public DeviceHandle {
public:
    NodeHandle();
    NodeHandle(const NodeInfo &node_info);
    NodeHandle(const DeviceHandle &device_handle, const unsigned short node_id);
    virtual ~NodeHandle();

public:
    unsigned short m_node_id;
};


struct CompareDeviceInfo
{
    bool operator()(const DeviceInfo &a, const DeviceInfo &b)
    {
        if (a.m_device_name != b.m_device_name)
        {
            return a.m_device_name < b.m_device_name;
        }
        if (a.m_protocol_stack != b.m_protocol_stack)
        {
            return a.m_protocol_stack < b.m_protocol_stack;
        }
        if (a.m_interface_name != b.m_interface_name)
        {
            return a.m_interface_name < b.m_interface_name;
        }
        return a.m_port_name < b.m_port_name;
    }
};


#endif // _Device_HPP
