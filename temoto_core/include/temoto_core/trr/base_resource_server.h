#ifndef TEMOTO_CORE__BASE_RESOURCE_SERVER_H
#define TEMOTO_CORE__BASE_RESOURCE_SERVER_H

#include "temoto_core/temoto_error/temoto_error.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/resource_registrar.h"
#include "temoto_core/UnloadResource.h"
#include <string>
#include <vector>
#include <utility>

namespace temoto_core
{
namespace trr
{

//Forward declatation of ResourceRegistrar
template<class Owner>
class ResourceRegistrar;

template<class Owner>
class BaseResourceServer : public BaseSubsystem
{
public:
  BaseResourceServer( const std::string& name
					, const std::string& class_name
					, ResourceRegistrar<Owner>& resource_registrar)
  : BaseSubsystem( resource_registrar.subsystem_name_ + "/" + name
  , resource_registrar.subsystem_code_
  , class_name
  , resource_registrar.log_group_)
  , name_(name)
  , resource_registrar_(resource_registrar)
  {}
  
  virtual ~BaseResourceServer()
  {}
  
  virtual void setFailedFlag(temoto_id::ID internal_resource_id, error::ErrorStack& error_stack) = 0;
  
  const std::string& getName()
  {
  	return name_;
  };

  #ifdef enable_tracing
  temoto_core::StringMap getTracerSpanContext()
  {
    return active_tracer_context_;
  }
  #endif

  virtual void linkInternalResource(temoto_id::ID resource_id) = 0;
  virtual void unlinkInternalResource(temoto_id::ID resource_id) = 0;
  virtual bool isLinkedTo(temoto_id::ID resource_id) const = 0;
  virtual bool hasInternalResource(temoto_id::ID resource_id) const = 0;
  virtual bool hasExternalResource(temoto_id::ID resource_id) const = 0;
  virtual void unloadResource(temoto_core::UnloadResource::Request& req, temoto_core::UnloadResource::Response& res) = 0;
  virtual std::vector<std::pair<temoto_id::ID, std::string>>
  getExternalResourcesByInternalId(temoto_id::ID internal_resource_id) = 0;
  virtual std::vector<std::pair<temoto_id::ID, std::string>>
  getExternalResourcesByExternalId(temoto_id::ID external_resource_id) = 0;

protected:
  void activateServer()
  {
  	resource_registrar_.setActiveServer(this);
  };
  
  void deactivateServer()
  {
  	resource_registrar_.setActiveServer(NULL);
  };
  
  ResourceRegistrar<Owner>& resource_registrar_;
  std::string name_;
  
  #ifdef enable_tracing
  temoto_core::StringMap active_tracer_context_;
  #endif
private:
};

} // trr namespace
} // temoto_core namespace

#endif