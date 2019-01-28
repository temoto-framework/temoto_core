#include "temoto_core/temoto_error/temoto_error.h"
#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/console_colors.h"

namespace temoto_core
{
namespace error
{

ErrorHandler::ErrorHandler()
{
}

ErrorHandler::ErrorHandler(Subsystem subsystem, std::string log_group)
  : subsystem_(subsystem), log_group_(log_group)
{
}

ErrorStack ErrorHandler::create(Code code, const std::string& prefix, const std::string& message) const
{
  ErrorStack est;

  temoto_core::Error error;
  error.subsystem = static_cast<int>(subsystem_);
  error.code = static_cast<int>(code);
  error.prefix = prefix;
  error.message = message;
  error.stamp = ros::Time::now();

// Print the message out if the verbose mode is enabled
#ifdef __TEMOTO_ERROR_HANDLER_VERBOSE__
  // TODO: Figure out how to get class_name_ here so we could use TEMOTO_ERROR_STREAM macro
  std::cout << CYAN << prefix << " ERROR: " << message << RESET <<std::endl;
#endif

  est.push_back(error);
  return est;
}


ErrorStack ErrorHandler::forward(ErrorStack error_stack, const std::string& prefix) const
{
  temoto_core::Error error;
  error.subsystem = static_cast<int>(subsystem_);
  error.code = static_cast<int>(Code::FORWARDING);
  error.prefix = prefix;
  error.stamp = ros::Time::now();

// Print the message out if the verbose mode is enabled
#ifdef __TEMOTO_ERROR_HANDLER_VERBOSE__
  //TEMOTO_ERROR_STREAM(prefix << " Forwarding.");
  std::cout << CYAN << prefix << " ERROR FWD: " << RESET << std::endl;
#endif

  error_stack.push_back(error);
  return error_stack;
}

void ErrorHandler::send(ErrorStack est) 
{
  // Create an ErrorStack message and publish it
  ros::Publisher error_publisher_ =
      n_.advertise<temoto_core::ErrorStack>("/temoto_core/temoto_error_messages", 100);

  while(error_publisher_.getNumSubscribers() < 1){}
  temoto_core::ErrorStack error_stack_msg;
  error_stack_msg.error_stack = est;
  error_publisher_.publish(error_stack_msg);
}


}  // error namespace
}  // temoto_core namespace

temoto_core::error::ErrorStack& operator+=(temoto_core::error::ErrorStack& es_lhs, const temoto_core::error::ErrorStack& es_rhs)
{
  es_lhs.insert(es_lhs.end(), es_rhs.begin(), es_rhs.end());
  return es_lhs;
}


std::ostream& operator<<(std::ostream& out, const temoto_core::Error& t)
{
  out << std::endl;
  out << "* code: " << t.code << std::endl;
  out << "* subsystem: " << t.subsystem << std::endl;
  out << "* prefix: " << t.prefix << std::endl;
  out << RED << "* message: " << t.message << RESET << std::endl;
  out << "* timestamp: " << t.stamp << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const temoto_core::error::ErrorStack& t)
{
  out << std::endl;
  // Start printing out the errors
  for (auto& err : t)
  {
    if (err.code != 0)
    {
      out << std::endl << " ------- Error Trace --------";
    }
    out << err;
  }
  return out;
}
