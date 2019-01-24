#ifndef TEMOTO_CORE__RMP_CONFIG_SYNCHRONIZER_H
#define TEMOTO_CORE__RMP_CONFIG_SYNCHRONIZER_H

#include "ros/ros.h"
#include <ros/serialization.h>
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/ConfigSync.h"
#include <string>
#include <sstream>

namespace ser = ros::serialization;

namespace temoto_core
{
namespace rmp
{

namespace sync_action
{
const std::string ADVERTISE_CONFIG = "advertise_config";
const std::string ADD_CONFIG = "add_config";
const std::string REMOVE_CONFIG = "remove_config";
const std::string REQUEST_CONFIG = "request_config";
}


template <class Owner, class PayloadType>
class ConfigSynchronizer : public BaseSubsystem
{
public:
  typedef void (Owner::*OwnerCbType)(const temoto_core::ConfigSync&, const PayloadType& payload);

  ConfigSynchronizer( const std::string& name
                    , const std::string& sync_topic
                    , OwnerCbType sync_cb
                    , Owner* owner)
    : name_(name)
    , sync_topic_(sync_topic)
    , sync_cb_(sync_cb)
    , owner_(owner)
    , BaseSubsystem(*owner, __func__)
  {
    // Setup publisher and subscriber
    sync_pub_ = nh_.advertise<temoto_core::ConfigSync>(sync_topic, 1000);
    sync_sub_ = nh_.subscribe(sync_topic, 1000, &ConfigSynchronizer::wrappedSyncCb, this);

    // Ask from master how many nodes have subscribed to sync_topic
    // Wait until all connections are established.
    int total_connections = 0;
    int active_connections = 0;
    while (true)
    {
      XmlRpc::XmlRpcValue args, result, payload;
      args[0] = ros::this_node::getName();
      ros::S_string node_set;
      if (ros::master::execute("getSystemState", args, result, payload, true))
      {
        for (int i = 0; i < payload.size(); ++i)
        {
          for (int j = 0; j < payload[i].size(); ++j)
          {
            std::string topic = payload[i][j][0];
            XmlRpc::XmlRpcValue nodes = payload[i][j][1];
            if (topic == sync_topic_)
            {
              total_connections = nodes.size();
            }
          }
        }
      }
      active_connections = sync_pub_.getNumSubscribers();
      TEMOTO_DEBUG("Waiting for subscribers: %d/%d", active_connections, total_connections);
      if (active_connections == total_connections)
      {
        break;
      }
      ros::Duration(0.1).sleep();
    }

    TEMOTO_INFO("ConfigSynchronizer created.");
  }

  ~ConfigSynchronizer()
  {
    sync_pub_.shutdown();
    sync_sub_.shutdown();
  }

  /**
   * @brief requestRemoteConfigs
   */
  void requestRemoteConfigs()
  {
    temoto_core::ConfigSync msg;
    msg.temoto_namespace = common::getTemotoNamespace();
    msg.action = sync_action::REQUEST_CONFIG;
    sync_pub_.publish(msg);
  }

  /**
   * @brief Advertise payload to others
   * @param payload
   */
  void advertise(const PayloadType& payload, std::string sync_action = sync_action::ADVERTISE_CONFIG) const
  {
    try
    {
      temoto_core::ConfigSync msg;
      msg.temoto_namespace = common::getTemotoNamespace();
      msg.action = sync_action;

      // Serialize the payload
      uint32_t payload_size = ros::serialization::serializationLength(payload);
      boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
      ser::OStream stream(buffer.get(), payload_size);
      ser::serialize(stream, payload);

      // Create a byte array
      std::vector<uint8_t> payload_byte_array;

      // Fill out the byte array
      for (uint32_t i=0; i<payload_size; i++)
      {
        payload_byte_array.push_back(buffer.get()[i]);
      }

      msg.payload = payload_byte_array;
      sync_pub_.publish(msg);
    }

    catch (...)
    {
      TEMOTO_ERROR("Siit lendas laiali @ advertise");
    }

  }

private:

  void wrappedSyncCb(const temoto_core::ConfigSync& msg)
  {
    // Ignore messages that are ours
    if (msg.temoto_namespace == common::getTemotoNamespace())
    {
      return;
    }

    try
    {
      // Create empty payload msg
      PayloadType payload;

      // Deserialize the payload if the action type is ADVERTISE
      if (msg.action == sync_action::ADVERTISE_CONFIG)
      {
        uint32_t payload_size = msg.payload.size();
        boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

        // Fill buffer with the serialized payload
        for (uint32_t i=0; i<payload_size; i++)
        {
          (buffer.get())[i] = msg.payload[i];
        }

        // Convert the serialized payload to msg
        ser::IStream stream(buffer.get(), payload_size);
        ser::deserialize(stream, payload);
      }

      (owner_->*sync_cb_)(msg, payload);
    }
    catch(error::ErrorStack& error_stack)
    {
      SEND_ERROR(FORWARD_ERROR(error_stack));
    }
  }

  std::string name_;
  std::string sync_topic_;
  OwnerCbType sync_cb_;
  Owner* owner_;

  ros::NodeHandle nh_;
  ros::Publisher sync_pub_;
  ros::Subscriber sync_sub_;
};

} // rmp namespace
} // temoto_core namespace

#endif