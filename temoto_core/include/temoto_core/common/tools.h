#ifndef TEMOTO_CORE__TOOLS_H
#define TEMOTO_CORE__TOOLS_H

#include <ros/ros.h>
#include <string>
#include <stdlib.h>  // for getenv

namespace temoto_core
{
namespace common
{
/**
 * @brief Used for createing a prefix that indicates where the message is coming from
 * @param class_name
 * @param method_name
 * @return
 */
inline std::string generateLogPrefix(std::string subsys_name, std::string class_name,
                                     std::string method_name)
{
  std::string prefixx = "[";
  if (subsys_name.size())
  {
    prefixx += subsys_name + "/";
  }
  if (class_name.size())
  {
    prefixx += class_name;
    if (method_name.size())
    {
      prefixx += "::";
    }
  }
  prefixx += method_name + "]";
  return prefixx;
}

inline const std::string getTemotoNamespace()
{
  //  std::string temoto_name_str;
  //
  //  char* temoto_name_char = NULL;
  //  temoto_name_char = getenv("TEMOTO_NAMESPACE");
  //  if (temoto_name_char != NULL)
  //  {
  //    temoto_name_str = temoto_name_char;
  //  }
  //  else
  //  {
  //    temoto_name_str = "temoto_core_default";
  //  }

  std::string temoto_ns = ::ros::this_node::getNamespace();
  temoto_ns = ::ros::names::clean(temoto_ns);  // clean up double and trailing slashes
  if (temoto_ns.size() > 0 and temoto_ns[0] == '/')
  {
    temoto_ns.erase(temoto_ns.begin());  // remove the leading '/'
  }

  return temoto_ns;
}

inline std::string getAbsolutePath(const std::string& path_in)
{
  std::string abs_path;
  if (path_in.size())
  {
    if (path_in[0] == '/')
    {
      // Specified topic is already absolute
      abs_path = path_in;
    }
    else
    {
      // Add current namespace as a prefix.
      abs_path = '/' + getTemotoNamespace() + '/' + path_in;
    }
  }
  return abs_path;
}

} // common namespace
} // temoto_core namespace
#endif
