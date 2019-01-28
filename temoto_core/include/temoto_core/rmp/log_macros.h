#ifndef TEMOTO_CORE__RMP_LOG_MACROS_H
#define TEMOTO_CORE__RMP_LOG_MACROS_H

// basic log management, everything put under temoto_core.tasks for easier level control
#define RMP_CONSOLE_PREFIX ROSCONSOLE_ROOT_LOGGER_NAME "." + ::temoto_core::common::getTemotoNamespace()  + "."+log_subsys_
#define RMP_DEBUG(...) ROS_LOG(::ros::console::levels::Debug, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_INFO(...) ROS_LOG(::ros::console::levels::Info, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_WARN(...) ROS_LOG(::ros::console::levels::Warn, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_ERROR(...) ROS_LOG(::ros::console::levels::Error, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_DEBUG_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Debug, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_INFO_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Info, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_WARN_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Warn, RMP_CONSOLE_PREFIX, __VA_ARGS__)
#define RMP_ERROR_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Error, RMP_CONSOLE_PREFIX, __VA_ARGS__)

#endif
