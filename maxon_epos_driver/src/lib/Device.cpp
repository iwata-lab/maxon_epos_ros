/**
 * @file   Device
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 12:19:52
 */

#include "maxon_epos_driver/Device.hpp"
#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/utils/Macros.hpp"
#include "maxon_epos_driver/utils/EposException.hpp"

#include <ros/ros.h>
#include <map>
#include <boost/foreach.hpp>


// =============================================================================
// DeviceInfo
// =============================================================================


/**
 * @brief Constructor
 */
DeviceInfo::DeviceInfo() : device_name(), protocol_stack(), interface_name(), port_name()
{}

/**
 * @brief Constructor
 *
 * @param device_name
 * @param protocol_stack
 * @param interface_name
 * @param portname
 */
DeviceInfo::DeviceInfo(const std::string &device_name, const std::string &protocol_stack,
                       const std::string &interface_name, const std::string &portname)
    : device_name(device_name), protocol_stack(protocol_stack), interface_name(interface_name), port_name(portname)
{}

/**
 * @brief Destructor
 */
DeviceInfo::~DeviceInfo()
{}


// =============================================================================
// DeviceHandle
// =============================================================================

/**
 * @brief Constructor
 */
DeviceHandle::DeviceHandle() : ptr()
{}

/**
 * @brief Constructor
 *
 * @param device_info DeviceInfo object
 */
DeviceHandle::DeviceHandle(const DeviceInfo &device_info) : ptr(MakePtr(device_info))
{}

/**
 * @brief Destructor
 */
DeviceHandle::~DeviceHandle()
{}

/**
 * @brief Create device ptr if not exist, otherwise return existing ptr
 *
 * @param device_info Device information to create device handle
 *
 * @return shared_ptr of device ptr
 */
std::shared_ptr<void> DeviceHandle::MakePtr(const DeviceInfo &device_info)
{
    static std::map<DeviceInfo, std::weak_ptr<void>, CompareDeviceInfo> existing_device_ptrs;

    // try to find an existing device
    const std::shared_ptr<void> existing_device_ptr(existing_device_ptrs[device_info].lock());
    if (existing_device_ptr)
    {
        return existing_device_ptr;
    }

    // open new device if not exist
    // Deleter is CloseDevice method
    const std::shared_ptr<void> new_device_ptr(OpenDevice(device_info), CloseDevice);
    existing_device_ptrs[device_info] = new_device_ptr;
    return new_device_ptr;
}

/**
 * @brief Open Epos Device
 *
 * @param device_info
 *
 * @return raw pointer of epos device
 */
void* DeviceHandle::OpenDevice(const DeviceInfo &device_info)
{
    unsigned int error_code;
    void* raw_device_ptr(
            VCS_OpenDevice(const_cast<char*>(device_info.device_name.c_str()),
                           const_cast<char*>(device_info.protocol_stack.c_str()),
                           const_cast<char*>(device_info.interface_name.c_str()),
                           const_cast<char*>(device_info.port_name.c_str()), &error_code));
    if (!raw_device_ptr) {
        throw EposException("OpenDevice", error_code);
    }
    return raw_device_ptr;
}

/**
 * @brief Close Epos Device
 *
 * @param raw_device_ptr Epos Device Handle
 */
void DeviceHandle::CloseDevice(void* raw_device_ptr)
{
    unsigned int error_code;
    if (VCS_CloseDevice(raw_device_ptr, &error_code) == VCS_FALSE) {
        ROS_ERROR_STREAM("CloseDevice (" + EposException::ToErrorInfo(error_code) + ")");
    }
}


// =============================================================================
// NodeInfo
// =============================================================================

/**
 * @brief Constructor
 */
NodeInfo::NodeInfo() : DeviceInfo(), node_id(0)
{}

/**
 * @brief Constructor
 *
 * @param device_info Device Information
 * @param node_id Node ID
 */
NodeInfo::NodeInfo(const DeviceInfo &device_info, const unsigned short node_id)
    : DeviceInfo(device_info), node_id(node_id)
{}

/**
 * @brief Destructor
 */
NodeInfo::~NodeInfo()
{}


// =============================================================================
// NodeHandle
// =============================================================================

/**
 * @brief Constructor
 */
NodeHandle::NodeHandle() : DeviceHandle(), node_id()
{}

/**
 * @brief Constructor
 *
 * @param node_info Node information
 */
NodeHandle::NodeHandle(const NodeInfo &node_info)
    : DeviceHandle(node_info), node_id(node_info.node_id)
{}


/**
 * @brief Constructor
 *
 * @param device_handle Device Handle obj
 * @param node_id Node ID
 */
NodeHandle::NodeHandle(const DeviceHandle &device_handle, unsigned short node_id)
    : DeviceHandle(device_handle), node_id(node_id)
{}

/**
 * @brief Destructor
 */
NodeHandle::~NodeHandle()
{}


// =============================================================================
// Utils
// =============================================================================

/**
 * @brief Create Epos Node Handle
 *
 * @param device_info DeviceInfo object
 * @param node_id node_id of EPOS which is not 0
 *
 * @return created NodeHandle object
 */
NodeHandle CreateEposHandle(const DeviceInfo &device_info, const unsigned short node_id)
{
    // TODO: support node_id == 0
    if (node_id == 0) {
        throw EposException("Invalid node_id");
    }

    try {
        NodeInfo node_info(device_info, node_id);
        NodeHandle node_handle(node_info);
        return node_handle;
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
        throw  EposException("CreateEposHandle (Could not identify node)");
    }
}
