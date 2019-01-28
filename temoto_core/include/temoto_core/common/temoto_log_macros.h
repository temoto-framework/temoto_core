#ifndef TEMOTO_CORE__TEMOTO_LOG_MACROS_H
#define TEMOTO_CORE__TEMOTO_LOG_MACROS_H

#include "temoto_core/common/tools.h"
#include <string>

#define TEMOTO_CONSOLE_NAME ROSCONSOLE_ROOT_LOGGER_NAME "."+::temoto_core::common::getTemotoNamespace()+"."+this->log_group_
#define TEMOTO_DEBUG(...) TEMOTO_LOG(::ros::console::levels::Debug, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_INFO(...) TEMOTO_LOG(::ros::console::levels::Info, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_WARN(...) TEMOTO_LOG(::ros::console::levels::Warn, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_ERROR(...) TEMOTO_LOG(::ros::console::levels::Error, TEMOTO_CONSOLE_NAME, __VA_ARGS__)

#define TEMOTO_DEBUG_STREAM(...) TEMOTO_LOG_STREAM(::ros::console::levels::Debug, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_INFO_STREAM(...) TEMOTO_LOG_STREAM(::ros::console::levels::Info, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_WARN_STREAM(...) TEMOTO_LOG_STREAM(::ros::console::levels::Warn, TEMOTO_CONSOLE_NAME, __VA_ARGS__)
#define TEMOTO_ERROR_STREAM(...) TEMOTO_LOG_STREAM(::ros::console::levels::Error, TEMOTO_CONSOLE_NAME, __VA_ARGS__)

#define TEMOTO_LOG_PREFIX ("::"+::temoto_core::common::getTemotoNamespace()+"/"+this->subsystem_name_+"/"+this->class_name_+"::"+__func__).c_str()

#define TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(filter, ...) \
::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __FILE__, __LINE__, TEMOTO_LOG_PREFIX, __VA_ARGS__)

#define TEMOTO_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args) \
  do \
  { \
    std::stringstream __rosconsole_print_stream_at_location_with_filter__ss__; \
    __rosconsole_print_stream_at_location_with_filter__ss__ << args; \
    ::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __rosconsole_print_stream_at_location_with_filter__ss__, __FILE__, __LINE__, TEMOTO_LOG_PREFIX); \
  } while (0)
  
#define TEMOTO_PRINT_AT_LOCATION(...) \
  TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(0, __VA_ARGS__)

#define TEMOTO_PRINT_STREAM_AT_LOCATION(args) \
  TEMOTO_PRINT_STREAM_AT_LOCATION_WITH_FILTER(0, args)

#define TEMOTO_LOG_FILTER(filter, level, name, ...) \
do \
{ \
  ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
  if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (filter)->isEnabled()) \
  { \
    TEMOTO_PRINT_AT_LOCATION_WITH_FILTER(filter, __VA_ARGS__); \
  } \
} while(0)
 
#define TEMOTO_LOG_COND(cond, level, name, ...) \
  do \
  { \
   ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
   \
   if (ROS_UNLIKELY(__rosconsole_define_location__enabled)) \
   { \
    TEMOTO_PRINT_AT_LOCATION(__VA_ARGS__); \
   } \
  } while(0)

#define TEMOTO_LOG_STREAM_COND(cond, level, name, args) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    if (ROS_UNLIKELY(__rosconsole_define_location__enabled)) \
    { \
      TEMOTO_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(false)

#define TEMOTO_LOG(level, name, ...) TEMOTO_LOG_COND(true, level, name, __VA_ARGS__)

#define TEMOTO_LOG_STREAM(level, name, args) TEMOTO_LOG_STREAM_COND(true, level, name, args)

#endif
