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
#include <memory>
#include <map>
#include "utils/Macros.hpp"

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
    DeviceHandle(const DeviceInfo& info, const std::shared_ptr<void> master_device_ptr);
    virtual ~DeviceHandle();

private:
    static std::shared_ptr<void> MakePtr(const DeviceInfo &device_info);
    static std::shared_ptr<void> MakeSubPtr(const DeviceInfo &device_info, const std::shared_ptr<void> master_device_ptr);
    static void* OpenDevice(const DeviceInfo &device_info);
    static void* OpenSubDevice(const DeviceInfo &device_info, const std::shared_ptr<void> master_device_ptr);
    static void CloseDevice(void *raw_device_ptr);
    static void CloseSubDevice(void *raw_device_ptr);

public:
    std::shared_ptr<void> ptr;
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
    NodeHandle(const NodeInfo &node_info, const DeviceHandle &device_handle);
    NodeHandle(const DeviceHandle &device_handle, const unsigned short node_id);
    virtual ~NodeHandle();

public:
    unsigned short node_id;
};

/**
 * @brief DeviceInfo Comparison
 */
struct CompareNodeInfo
{
    bool operator()(const NodeInfo &a, const NodeInfo &b) const
    {
        if (a.device_name != b.device_name) {
            return a.device_name < b.device_name;
        }
        if (a.protocol_stack != b.protocol_stack) {
            return a.protocol_stack < b.protocol_stack;
        }
        if (a.interface_name != b.interface_name) {
            return a.interface_name < b.interface_name;
        }
        if (a.port_name != b.port_name) {
            return a.port_name < b.port_name;
        }
        return a.node_id < b.node_id;
    }
};


class HandleManager {
public:
    HandleManager();
    ~HandleManager();
    static NodeHandle CreateEposHandle(const DeviceInfo &device_info, const unsigned short node_id);

private:
    static std::shared_ptr<NodeHandle> m_master_handle;
    static std::vector<std::shared_ptr<NodeHandle>> m_sub_handles;
};


#endif // _Device_HPP
