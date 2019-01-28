#ifndef TEMOTO_CORE__CLIENT_QUERY_H
#define TEMOTO_CORE__CLIENT_QUERY_H

#include "temoto_core/common/temoto_id.h"
#include <string>
#include <vector>
#include <map>
#include <utility>

namespace temoto_core
{
namespace rmp
{

enum class FailureBehavior : int
{
  NONE,
  UNLOAD,
  RELOAD
};

// class for storing resource requests and hold their bingdings to external servers
template <class ServiceMsgType, class Owner>
class ClientQuery : public BaseSubsystem
{
public:
  // special constructor for resource client
  ClientQuery(const ServiceMsgType& msg, Owner* owner)
    : BaseSubsystem(*owner, __func__), msg_(msg), owner_(owner), failed_(false)
  {
    this->log_group_ = "rmp." + this->log_group_;
  }

  void addInternalResource(temoto_id::ID resource_id, FailureBehavior failure_behavior)
  {
    // try to insert to map
    TEMOTO_DEBUG("Adding internal resource, id:%d", resource_id);
    auto ret = internal_resources_.emplace(resource_id, failure_behavior);

    if (!ret.second)
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL, "Not allowed to add internal resources with "
                                                 "identical ids.");
    }
  }

  // remove the internal resource from this query and return how many are still connected
  void removeInternalResource(temoto_id::ID resource_id)
  {
    auto it = internal_resources_.find(resource_id);
    if (it == internal_resources_.end())
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL,
                         "Unable to remove the resource. Resource_id %s not found.", resource_id);
    }

    /// Erase resource_id from internal resource map.
    internal_resources_.erase(it);
  }

  // Check if given internal resource_id is attached to this query.
  bool internalResourceExists(temoto_id::ID resource_id) const
  {
    return internal_resources_.find(resource_id) != internal_resources_.end();
  }

  const ServiceMsgType& getMsg() const
  {
    return msg_;
  }

  const temoto_id::ID getExternalId() const
  {
    return msg_.response.rmp.resource_id;
  }

  std::string toString() const
  {
    std::stringstream ret;
    ret << "   Query req:" << std::endl << msg_.request << std::endl;
    ret << "   Query res:" << std::endl << msg_.response << std::endl;
    ret << "   Internal resources:" << std::endl;
    std::string behavior_string;
    for (auto& r : internal_resources_)
    {
      switch (r.second)
      {
        case FailureBehavior::UNLOAD:
          behavior_string = "UNLOAD";
          break;
        case FailureBehavior::RELOAD:
          behavior_string = "RELOAD";
          break;
        default:
          behavior_string = "NONE";
      }
      ret << "id:" << r.first << "; FailureBehavior: '" << behavior_string << "'";
    }
    return ret.str();
  }

  const std::map<temoto_id::ID, FailureBehavior> getInternalResources() const
  {
    return internal_resources_;
  }

  bool failed_;

private:
  // internal resource ids and their callers name
  std::map<temoto_id::ID, FailureBehavior> internal_resources_;

  Owner* owner_;

  ServiceMsgType msg_;  /// Store request and response, note that RMP specific fields (resource_id,
                        /// topic, ...) are related to first query and are not intended to be used
                        /// herein.
};

} // rmp namespace
} // temoto_core namespace

#endif
