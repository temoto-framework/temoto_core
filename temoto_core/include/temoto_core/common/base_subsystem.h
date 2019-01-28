#ifndef TEMOTO_CORE__BASE_SUBSYSTEM_H
#define TEMOTO_CORE__BASE_SUBSYSTEM_H

#include "temoto_core/temoto_error/temoto_error.h"
#include <string>

namespace temoto_core
{
class BaseSubsystem
{
public:
  BaseSubsystem() = default;

  // Constructor without log group
  BaseSubsystem(std::string subsystem_name, error::Subsystem subsystem_code, std::string class_name)
    : BaseSubsystem(subsystem_name, subsystem_code, class_name, subsystem_name)
  {
  }

  // Constructor with all arguments
  BaseSubsystem(std::string subsystem_name, error::Subsystem subsystem_code, std::string class_name,
                std::string log_group)
    : subsystem_name_(subsystem_name)
    , subsystem_code_(subsystem_code)
    , class_name_(class_name)
    , log_group_(log_group)
    , error_handler_(error::ErrorHandler(subsystem_code, log_group))
  {
  }

  // Copy constructor
  BaseSubsystem(const BaseSubsystem& b) : BaseSubsystem(b, b.class_name_)
  {
  }

  // Copy constructor with class name
  BaseSubsystem(const BaseSubsystem& b, const std::string& class_name)
    : subsystem_name_(b.subsystem_name_)
    , subsystem_code_(b.subsystem_code_)
    , class_name_(class_name)
    , log_group_(b.log_group_)
    , error_handler_(error::ErrorHandler(b.subsystem_code_, b.log_group_))
  {
  }

protected:
  std::string subsystem_name_;
  error::Subsystem subsystem_code_;
  std::string class_name_;
  std::string log_group_;
  error::ErrorHandler error_handler_;

  /**
   * @brief This function is used when the BaseSubsystem cannot be initialized
   * during the construction phase
   * @param b
   */
  void initializeBase(const BaseSubsystem* b)
  {
    subsystem_name_ = b->subsystem_name_;
    subsystem_code_ = b->subsystem_code_;
    log_group_ = b->log_group_;
    error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);
  }

  std::string generateLogPrefix(const std::string& func_name)
  {
    return common::generateLogPrefix(subsystem_name_, class_name_, func_name);
  }
};

} // temoto_core namespace
#endif
