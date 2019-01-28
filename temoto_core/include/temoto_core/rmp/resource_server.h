#ifndef TEMOTO_CORE__RESOURCE_SERVER_H
#define TEMOTO_CORE__RESOURCE_SERVER_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/tools.h"
#include "temoto_core/rmp/base_resource_server.h"
#include "temoto_core/rmp/server_query.h"
#include "temoto_core/rmp/resource_manager_services.h"
#include <mutex>

namespace temoto_core
{
namespace rmp
{

template <class ServiceType, class Owner>
class ResourceServer : public BaseResourceServer<Owner>
{
public:
  typedef void (Owner::*LoadCbFuncType)(typename ServiceType::Request&,
                                        typename ServiceType::Response&);
  typedef void (Owner::*UnloadCbFuncType)(typename ServiceType::Request&,
                                          typename ServiceType::Response&);

  ResourceServer(std::string name, LoadCbFuncType load_cb, UnloadCbFuncType unload_cb, Owner* owner,
                 ResourceManager<Owner>& resource_manager)
    : BaseResourceServer<Owner>(name, resource_manager)
    , load_callback_(load_cb)
    , unload_callback_(unload_cb)
    , owner_(owner)
    , load_spinner_(2, &load_cb_queue_)

  {
    this->class_name_ = __func__;

    std::string rm_name = this->resource_manager_.getName();
    std::string server_srv_name = rm_name + "/" + this->name_;

    ros::AdvertiseServiceOptions load_service_opts =
        ros::AdvertiseServiceOptions::create<ServiceType>(
            server_srv_name,
            boost::bind(&ResourceServer<ServiceType, Owner>::wrappedLoadCallback, this, _1, _2),
            ros::VoidPtr(), &this->load_cb_queue_);

    load_server_ = nh_.advertiseService(load_service_opts);
    load_spinner_.start();
    TEMOTO_DEBUG("ResourceServer constructed, listening on '%s'.", this->load_server_.getService().c_str());
  }

  ~ResourceServer()
  {
    load_spinner_.stop();
    TEMOTO_DEBUG("ResourceServer destroyed.");
  }

  void linkInternalResource(temoto_id::ID internal_resource_id)
  {
    TEMOTO_DEBUG("Trying to register client side id in resource query %d.", internal_resource_id);
    if (!queries_.size())
    {
      TEMOTO_ERROR("Failed because queries_ is empty.");
      return;
    }
    queries_.back().linkResource(internal_resource_id);
  }

  
  void unlinkInternalResource(temoto_id::ID internal_resource_id)
  {
    TEMOTO_DEBUG("Trying to unlink resource '%d'.", internal_resource_id);
    try
    {
      const auto found_query_it =
          std::find_if(queries_.begin(), queries_.end(),
                       [internal_resource_id](const ServerQuery<ServiceType>& query) -> bool {
                         return query.isLinkedTo(internal_resource_id);
                       });
      if (found_query_it != queries_.end())
      {
        found_query_it->unlinkResource(internal_resource_id);
        //if(found_query_it->getInternalResources().size()==0)
      }
      else
      {
        throw CREATE_ERROR(error::Code::RMP_FAIL,
                           "Resource id '%ld' was not found from any queries.",
                           internal_resource_id);
      }
    }
    catch (error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  bool wrappedLoadCallback(typename ServiceType::Request& req, typename ServiceType::Response& res)
  {
    TEMOTO_DEBUG("Got query with status_topic: '%s'.", req.rmp.status_topic.c_str());

    if (!owner_)
    {
      res.rmp.code = status_codes::FAILED;
      res.rmp.error_stack =
          CREATE_ERROR(error::Code::RMP_FAIL, "ResourceServer Owner is NULL. Query aborted.");
      return true;
    }

    // generate new external id for the resource
    temoto_id::ID ext_resource_id = this->resource_manager_.generateID();
    TEMOTO_DEBUG("Generated external id: '%d'.", ext_resource_id);

    // lock the queries
    waitForLock(queries_mutex_);

    // New or existing query? Check it out with this hi-tec lambda function :)
    auto found_query = std::find_if(queries_.begin(), queries_.end(),
                                    [&req](const ServerQuery<ServiceType>& query) -> bool {
                                      return query.getMsg().request == req && !query.failed_;
                                    });

    if (found_query == queries_.end())
    {
      // generate new internal id, and give it to owners callback.
      // with this id, owner can send status messages later when necessary
      temoto_id::ID int_resource_id = this->resource_manager_.generateID();
      res.rmp.resource_id = int_resource_id;
      TEMOTO_DEBUG("New query, server generated new internal id: '%d'.", int_resource_id);

      // equal message not found from queries_, add new query
      try
      {
        queries_.emplace_back(req, int_resource_id, *owner_);
        queries_.back().addExternalResource(ext_resource_id, req.rmp.status_topic);
      }
      catch(error::ErrorStack& error_stack)
      {
        queries_.pop_back(); //remove the failed query
        queries_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack = FORWARD_ERROR(error_stack);
        return true;
      }

      // set this server active in resource manager
      // when a client call is made from callback, the binding between active server
      // and the new loaded resources can be made automatically
      waitForLock(active_server_mutex_);
      try
      {
        this->activateServer();
      }
      catch(error::ErrorStack& error_stack)
      {
        queries_.pop_back(); //remove the failed query
        queries_mutex_.unlock();
        active_server_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack = FORWARD_ERROR(error_stack);
        return true;
      }
      queries_mutex_.unlock();

      // call owner's registered callback and release the lock during the callback so that owner is
      // able to use rmp inside the callback
      try
      {
        (owner_->*load_callback_)(req, res);
      }
      catch(error::ErrorStack& error_stack)
      {
        // callback threw an exeption, try to clean up and return.
        try
        {
          waitForLock(queries_mutex_);
          auto q_it = getQueryByExternalId(ext_resource_id);
          if(q_it->failed_)
          {
            res.rmp.error_stack += q_it->getMsg().response.rmp.error_stack;
          }
          queries_.erase(q_it);
          queries_mutex_.unlock();
        }
        catch(error::ErrorStack& query_error)
        {
          // just send the error when trying to  and continue
          queries_mutex_.unlock();
          SEND_ERROR(FORWARD_ERROR(query_error));
        }

        this->deactivateServer();
        active_server_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack = FORWARD_ERROR(error_stack);
        return true;
      }
      catch(...)
      {
        // callback threw an unknown exeption, prevent this for reaching ros callback.
        // try to clean up and report.
        try
        {
          waitForLock(queries_mutex_);
          auto q_it = getQueryByExternalId(ext_resource_id);
          if(q_it->failed_)
          {
            res.rmp.error_stack += q_it->getMsg().response.rmp.error_stack;
          }
          queries_.erase(q_it);
          queries_mutex_.unlock();
        }
        catch(error::ErrorStack& query_error)
        {
          // just send the error when trying to  and continue
          queries_mutex_.unlock();
          SEND_ERROR(FORWARD_ERROR(query_error));
        }
        this->deactivateServer();
        active_server_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack =
            CREATE_ERROR(error::Code::RMP_FAIL, "Unexpected error was thrown from "
                                                           "owner's load callback.");
        return true;
      }

      // restore active server to NULL in resource manager
      this->deactivateServer();
      active_server_mutex_.unlock();

      waitForLock(queries_mutex_);
      try
      {
        // verify that our query is still on the list
        auto q_it = std::find_if(queries_.begin(), queries_.end(),
            [&](const ServerQuery<ServiceType>& q) -> bool {
            return q.hasExternalResource(ext_resource_id);
            });
        if (q_it != queries_.end())
        {
          // First, make sure the internal_resource_id for that query is not changed.
          res.rmp.resource_id = int_resource_id;

          // check if the query has not been marked as failed while we were dealing with the owner's callback
          // if it failed, remove the query. 
          if(q_it->failed_)
          {
            // TODO Potentially some resources were sucessfully loaded, SEND UNLOAD REQUEST TO ALL
            // LINKED CLIENTS
            res.rmp.error_stack += q_it->getMsg().response.rmp.error_stack;
            queries_.erase(q_it);
            queries_mutex_.unlock();
            res.rmp.code = status_codes::FAILED;
            return true;
          }

          // update the query with the response message filled in the callback
          q_it->setMsgResponse(res);

          // prepare the response for the client
          res.rmp.resource_id = ext_resource_id;

          //res.rmp.code = status_codes::OK;
          //res.rmp.message = "New resource sucessfully loaded.";
        }
        else
        {
          queries_mutex_.unlock();
          res.rmp.code = status_codes::FAILED;
          res.rmp.error_stack = CREATE_ERROR(error::Code::RMP_FAIL, "Query got missing during owners callback, oh well...");
          return true;
        }
      }
      catch(error::ErrorStack& error_stack)
      {
        queries_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack = FORWARD_ERROR(error_stack);
        return true;
      }

    }
    else
    {
      try
      {
        // found equal request, simply reqister this in the query
        // and respond with previous data and a unique resoure_id.
        TEMOTO_DEBUG("Existing query, linking to the found query.");
        queries_.back().addExternalResource(ext_resource_id, req.rmp.status_topic);
        res = found_query->getMsg().response;
        res.rmp.resource_id = ext_resource_id;
        //res.rmp.code = status_codes::OK;
        //res.rmp.message = "Sucessfully sharing existing resource.";
      }
      catch(error::ErrorStack& error_stack)
      {
        queries_mutex_.unlock();
        res.rmp.code = status_codes::FAILED;
        res.rmp.error_stack = FORWARD_ERROR(error_stack);
        return true;
      }
    }

    // release the queries lock
    queries_mutex_.unlock();

    return true;
  }

  // This function is called from resource manager when /unload request arrives
  // (e.g. when some external client is being destroyed)
  // We look up the query that contains given external resource id and send unload to all internal
  // clients in the same query

  void unloadResource(temoto_core::UnloadResource::Request& req,
                      temoto_core::UnloadResource::Response& res)
  {
    // find first query that contains resource that should be unloaded
    const temoto_id::ID ext_rid = req.resource_id;
    waitForLock(queries_mutex_);
    const auto found_query_it =
        std::find_if(queries_.begin(), queries_.end(),
                     [ext_rid](const ServerQuery<ServiceType>& query) -> bool {
                       return query.hasExternalResource(ext_rid);
                     });
    if (found_query_it != queries_.end())
    {

      TEMOTO_DEBUG("Query with ext id %d was found", ext_rid);
      TEMOTO_DEBUG("internal resource count: %lu", found_query_it->getLinkedResources().size());

      // Query found, try to remove external client from it.
      size_t resources_left = found_query_it->removeExternalResource(ext_rid);
      if (resources_left == 0)
      {
        // last resource removed, execute owner's unload callback and remove the query from our list
        typename ServiceType::Request orig_req = found_query_it->getMsg().request;
        typename ServiceType::Response orig_res = found_query_it->getMsg().response;
        error::ErrorStack unload_errs; // buffer for all unload-related errors
        try
        {
          (owner_->*unload_callback_)(orig_req, orig_res);
        }
        catch(error::ErrorStack& error_stack)
        {
          unload_errs += error_stack;
        }

        // Send unload command to all linked internal clients...
        for (auto& set_el : found_query_it->getLinkedResources())
        {
          try
          {
            this->resource_manager_.unloadClientResource(set_el);
          }
          catch (error::ErrorStack& es)
          {
            unload_errs += es; //append error to the unload error stack
          }
        }

        // forward the stack if any error occured
        if (unload_errs.size())
        {
            res.code = status_codes::FAILED;
            res.error_stack += FORWARD_ERROR(unload_errs);
        }

        // Finally, remove the found query, even when some of the unload calls failed.
        // \TODO: The next line potentially causes zombie resources. How to manage these?
        queries_.erase(found_query_it);
      }
    }
    queries_mutex_.unlock();
  }

  // go over queries_ and look for the linked resource_id
  bool hasInternalResource(temoto_id::ID resource_id) const
  {
    auto found_q = find_if(queries_.begin(), queries_.end(),
                           [&](const ServerQuery<ServiceType>& q) -> bool {
                             return q.hasInternalResource(resource_id);
                           });
    return found_q != queries_.end();
  }

  bool isLinkedTo(temoto_id::ID resource_id) const
  {
    auto found_q = find_if(queries_.begin(), queries_.end(),
                           [&](const ServerQuery<ServiceType>& q) -> bool {
                             return q.isLinkedTo(resource_id);
                           });
    return found_q != queries_.end();
  }

  bool hasExternalResource(temoto_id::ID external_resource_id) const
  {
    auto found_q = find_if(queries_.begin(), queries_.end(),
                           [&](const ServerQuery<ServiceType>& q) -> bool {
                             return q.hasExternalResource(external_resource_id);
                           });
    return found_q != queries_.end();
  }

  // concatenate pairs of <external_resource_id, status_topic> of the query where internal id is found.
  std::vector<std::pair<temoto_id::ID, std::string>>
  getExternalResourcesByInternalId(temoto_id::ID internal_resource_id)
  {
    waitForLock(queries_mutex_);

    std::vector<std::pair<temoto_id::ID, std::string>> ext_resources;
    for (const auto& q : queries_)
    {
      if (q.hasInternalResource(internal_resource_id))
      {
        for (auto resource : q.getExternalResources())
        {
          ext_resources.push_back(resource);
        }
        break; // no need to look any further
      }
    }
    queries_mutex_.unlock();
    return ext_resources;
  }
  
  // concatenate pairs of <external_resource_id, status_topic> of the query where external id is found.
  std::vector<std::pair<temoto_id::ID, std::string>>
  getExternalResourcesByExternalId(temoto_id::ID external_resource_id)
  {
    waitForLock(queries_mutex_);

    std::vector<std::pair<temoto_id::ID, std::string>> ext_resources;
    for (const auto& q : queries_)
    {
      if (q.hasExternalResource(external_resource_id))
      {
        for (auto resource : q.getExternalResources())
        {
          ext_resources.push_back(resource);
        }
        break;  // no need to look any further
      }
    }
    queries_mutex_.unlock();
    return ext_resources;
  }

  typename std::vector<ServerQuery<ServiceType>>::iterator getQueryByExternalId(temoto_id::ID ext_id)
  {
    auto q_it =std::find_if(queries_.begin(), queries_.end(), [&](const ServerQuery<ServiceType>& q) -> bool {
      return q.hasExternalResource(ext_id);
    });
    if (q_it == queries_.end())
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL, "External id not found from any queries.");
    }
    return q_it;
  }

  void waitForLock(std::mutex& m)
  {
    while (!m.try_lock())
    {
      TEMOTO_DEBUG("Waiting for lock()");
      ros::Duration(0.01).sleep();  // sleep for few ms
    }
  }

  void setFailedFlag(temoto_id::ID internal_resource_id, error::ErrorStack& error_stack)
  {
    waitForLock(queries_mutex_);
    const auto found_query_it =
        std::find_if(queries_.begin(), queries_.end(),
                     [internal_resource_id](const ServerQuery<ServiceType>& query) -> bool {
                       return query.hasInternalResource(internal_resource_id);
                     });
    if (found_query_it != queries_.end())
    {
      // Query found, mark query as failed and append why did it fail.
      found_query_it->setFailed(error_stack);
    }
    queries_mutex_.unlock();
  }

private:
  Owner* owner_;
  LoadCbFuncType load_callback_;
  UnloadCbFuncType unload_callback_;

  ros::NodeHandle nh_;
  ros::ServiceServer load_server_;
  ros::CallbackQueue load_cb_queue_;
  ros::AsyncSpinner load_spinner_;

  std::vector<ServerQuery<ServiceType>> queries_;

  // mutexes
  std::mutex queries_mutex_;
  std::mutex active_server_mutex_;
};

} // rmp namespace
} // temoto_core namespace

#endif
