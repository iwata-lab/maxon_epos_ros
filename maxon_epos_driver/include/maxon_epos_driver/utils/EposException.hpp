/**
 * @file   EposException
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:34:42
 */

#ifndef _EposException_HPP
#define _EposException_HPP

#include <stdexcept>


class EposException : public std::runtime_error {
public:
    EposException(const std::string &error_msg);
    EposException(const std::string &error_msg, const unsigned int error_code);
    virtual ~EposException() throw();

    bool HasErrorCode() const;
    unsigned int GetErrorCode() const;
    
    static std::string ToErrorInfo(const unsigned int error_code);

private:
    bool m_has_error_code;
    unsigned int m_error_code;
};

#endif // _EposException_HPP
