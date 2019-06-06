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

DeviceHandle::DeviceHandle(const DeviceInfo& info, const std::shared_ptr<void> master_device_ptr) : ptr(MakeSubPtr(info, master_device_ptr))
{}

/**
 * @brief Destructor
 */
DeviceHandle::~DeviceHandle()
{}

/**
 * @brief Create master device ptr
 *
 * @param device_info Device information to create device handle
 *
 * @return shared_ptr of device ptr
 */
std::shared_ptr<void> DeviceHandle::MakePtr(const DeviceInfo &device_info)
{
    // open new master device if not exist
    // Deleter is CloseDevice method
    const std::shared_ptr<void> new_device_ptr(OpenDevice(device_info), CloseDevice);
    return new_device_ptr;
}

/**
 * @brief Create sub device ptr
 *
 * @param device_info Device information to create device handle
 * @param master_device_ptr shared_ptr of master device
 *
 * @return shared_ptr of sub device ptr
 */
std::shared_ptr<void> DeviceHandle::MakeSubPtr(const DeviceInfo &device_info, const std::shared_ptr<void> master_device_ptr)
{
    // open new sub device
    // Deleter is CloseSubDevice method
    const std::shared_ptr<void> new_subdevice_ptr(OpenSubDevice(device_info, master_device_ptr), CloseSubDevice);
    return new_subdevice_ptr;
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
 * @brief Open Epos Sub Device
 *
 * @param device_info
 *
 * @return raw pointer of epos sub device
 */
void* DeviceHandle::OpenSubDevice(const DeviceInfo &device_info, const std::shared_ptr<void> master_device_ptr)
{
    unsigned int error_code;
    void* raw_subdevice_ptr(
            VCS_OpenSubDevice(master_device_ptr.get(),
                              const_cast<char*>(device_info.device_name.c_str()),
                              const_cast<char*>(device_info.protocol_stack.c_str()), &error_code));
    if (!raw_subdevice_ptr) {
        throw EposException("OpenSubDevice", error_code);
    }
    return raw_subdevice_ptr;
}

/**
 * @brief Close Epos Device
 *
 * @param raw_device_ptr Epos Device Handle
 */
void DeviceHandle::CloseDevice(void* raw_device_ptr)
{
    ROS_INFO("Called CloseDevice");
    unsigned int error_code;
    if (VCS_CloseDevice(raw_device_ptr, &error_code) == VCS_FALSE) {
        ROS_ERROR_STREAM("CloseDevice (" + EposException::ToErrorInfo(error_code) + ")");
    }
}

/**
 * @brief Close Epos Sub Device
 *
 * @param raw_device_ptr Epos Sub Device Handle
 */
void DeviceHandle::CloseSubDevice(void *raw_device_ptr)
{
    ROS_INFO("Called CloseSubDevice");
    unsigned int error_code;
    if (VCS_CloseSubDevice(raw_device_ptr, &error_code) == VCS_FALSE) {
        ROS_ERROR_STREAM("CloseSubDevice (" + EposException::ToErrorInfo(error_code) + ")");
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

NodeHandle::NodeHandle(const NodeInfo &node_info, const DeviceHandle &device_handle)
    : DeviceHandle(node_info, device_handle.ptr), node_id(node_info.node_id)
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
// HandleManager
// =============================================================================

/**
 * @brief entity of m_master_handle
 */
std::shared_ptr<NodeHandle> HandleManager::m_master_handle;
/**
 * @brief entity of m_sub_handles
 */
std::vector<std::shared_ptr<NodeHandle>> HandleManager::m_sub_handles;

/**
 * @brief Constructor
 */
HandleManager::HandleManager()
{}

/**
 * @brief Destructor
 */
HandleManager::~HandleManager()
{
    m_sub_handles.clear();
    m_master_handle.reset();
}

NodeHandle HandleManager::CreateEposHandle(const DeviceInfo &device_info, const unsigned short node_id)
{
    if (node_id == 0) {
        throw EposException("Invalid node_id");
    }

    static std::map<DeviceInfo, std::weak_ptr<NodeHandle>, CompareDeviceInfo> existing_node_handles;

    try {
        const std::shared_ptr<NodeHandle> existing_handle(existing_node_handles[device_info].lock());
        if (existing_handle) {
            return *existing_handle;
        }

        NodeInfo node_info(device_info, node_id);
        if (!m_master_handle) {
            // Create Master Handle
            m_master_handle = std::make_shared<NodeHandle>(NodeHandle(node_info));
            existing_node_handles[device_info] = m_master_handle;
            return *m_master_handle;
        } else {
            // Create Sub Handle
            const std::shared_ptr<NodeHandle> sub_handle = std::make_shared<NodeHandle>(NodeHandle(node_info, *m_master_handle));
            m_sub_handles.push_back(sub_handle);
            existing_node_handles[device_info] = sub_handle;
            return *sub_handle;
        }
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
        throw EposException("Create EposHandle (Could not identify node)");
    }
}
