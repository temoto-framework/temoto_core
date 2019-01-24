#ifndef TEMOTO_CORE__BASE_RESOURCE_SERVER_H
#define TEMOTO_CORE__BASE_RESOURCE_SERVER_H

#include "temoto_core/temoto_error/temoto_error.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/UnloadResource.h"
#include <string>
#include <vector>
#include <utility>

namespace temoto_core
{
namespace rmp
{

//Forward declatation of ResourceManager
template<class Owner>
class ResourceManager;

template<class Owner>
class BaseResourceServer : public BaseSubsystem
{
	public:
//		BaseResourceServer()
//		{
//			name_ = "default_server_name";
//			resource_manager_ = NULL;
//		}

		BaseResourceServer(const std::string name, ResourceManager<Owner>& resource_manager) :
      BaseSubsystem (resource_manager),
			name_(name),
			resource_manager_(resource_manager)
		{
		}
		virtual ~BaseResourceServer()
		{
		}

    virtual void setFailedFlag(temoto_id::ID internal_resource_id, error::ErrorStack& error_stack) = 0;

    const std::string& getName()
		{
			return name_;
		};

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
			resource_manager_.setActiveServer(this);
		};

		void deactivateServer()
		{
			resource_manager_.setActiveServer(NULL);
		};

		ResourceManager<Owner>& resource_manager_;
		std::string name_;


	private:
};

} // rmp namespace
} // temoto_core namespace

#endif