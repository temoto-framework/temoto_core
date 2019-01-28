#ifndef TEMOTO_CORE__RESOURCE_MANAGER_SERVICES_H
#define TEMOTO_CORE__RESOURCE_MANAGER_SERVICES_H

#include "temoto_core/UnloadResource.h"
#include "temoto_core/ResourceStatus.h"
#include "temoto_core/common/tools.h"

namespace temoto_core
{
namespace rmp
{

enum status_codes : int
{
  OK = 0,
  FAILED,
};

namespace srv_name
{
//const std::string PREFIX = ::temoto_core::common::getTemotoNamespace();
}

} // rmp namespace
} // temoto_core namespace

#endif
