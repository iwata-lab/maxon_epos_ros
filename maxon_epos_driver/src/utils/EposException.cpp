/**
 * @file   EposException
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:46:38
 */

#include "maxon_epos_driver/utils/EposException.hpp"
#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/utils/Macros.hpp"

#include <sstream>

/**
 * @brief Constructor
 */
EposException::EposException(const std::string &error_msg)
    : std::runtime_error(error_msg), m_has_error_code(false), m_error_code(0)
{}

/**
 * @brief Constructor
 */
EposException::EposException(const std::string &error_msg, const unsigned int error_code)
    : std::runtime_error(error_msg + " (" + ToErrorInfo(error_code) + ")"), m_has_error_code(true), m_error_code(error_code)
{}

/**
 * @brief Destructor
 */
EposException::~EposException() throw() {}

bool EposException::HasErrorCode() const
{
    return m_has_error_code;
}

unsigned int EposException::GetErrorCode() const
{
    return m_error_code;
}


std::string EposException::ToErrorInfo(const unsigned int error_code)
{
    std::ostringstream oss;
    oss << "0x" << std::hex << error_code;
    char error_info[1024];
    if (VCS_GetErrorInfo(error_code, error_info, 1024) != VCS_FALSE)
    {
        oss << ": " << error_info;
    }
    return oss.str();
}
