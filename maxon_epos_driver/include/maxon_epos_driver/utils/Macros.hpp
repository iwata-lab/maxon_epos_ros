/**
 * @file   Macros
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:54:56
 */

#ifndef _Macros_HPP
#define _Macros_HPP

#include "EposException.hpp"
#include "../Definitions.h"


// range of node_id [1, 127]
#define MAX_NODE_ID 127

// Epos Command False
#define VCS_FALSE 0

// Call VCS function
#define VCS_COMMAND(func, ...) \
    do { \
        unsigned int error_code; \
        if (VCS_##func(__VA_ARGS__, &error_code) == VCS_FALSE) { \
            throw EposException(#func, error_code); \
        } \
    } while(false)

// Call VCS function with Epos Device Handle
#define VCS_DEVICE_COMMAND(func, epos_device_handle, ...) \
    VCS_COMMAND(func, epos_device_handle.ptr.get(), __VA_ARGS__)

#define VCS_NODE_COMMAND_NO_ARGS(func, epos_node_handle) \
    VCS_DEVICE_COMMAND(func, epos_node_handle, epos_node_handle.node_id)

// Call VCS function with Epos Device Handle and Node ID
#define VCS_NODE_COMMAND(func, epos_node_handle, ...) \
    VCS_DEVICE_COMMAND(func, epos_node_handle, epos_node_handle.node_id, __VA_ARGS__)

#endif // _Macros_HPP
