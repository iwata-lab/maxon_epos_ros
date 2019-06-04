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
#include "maxon_epos_driver/utils/Macros.hpp"

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
    std::string device_name;
    std::string protocol_stack;
    std::string interface_name;
    std::string port_name;
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
    unsigned short node_id;
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
    unsigned short node_id;
};


/**
 * @brief DeviceInfo Comparison
 */
struct CompareDeviceInfo
{
    bool operator()(const DeviceInfo &a, const DeviceInfo &b)
    {
        if (a.device_name != b.device_name)
        {
            return a.device_name < b.device_name;
        }
        if (a.protocol_stack != b.protocol_stack)
        {
            return a.protocol_stack < b.protocol_stack;
        }
        if (a.interface_name != b.interface_name)
        {
            return a.interface_name < b.interface_name;
        }
        return a.port_name < b.port_name;
    }
};


NodeHandle CreateEposHandle(const DeviceInfo &device_info, const unsigned short node_id);
std::vector<NodeInfo> EnumerateNodes(const DeviceInfo &device_info, const unsigned short node_id,
                                     const unsigned short max_id = MAX_NODE_ID);


#endif // _Device_HPP