#include <opentracing/dynamic_load.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <string>
#include <text_map_carrier.h>

#include <ros/ros.h>
#include "temoto_core/Tracing.h"
#include "temoto_core/common/tracer_conversions.h"

using namespace opentracing;

class TracedService
{
public:
  /**
   * @brief Construct a new Traced Service object
   * 
   * @param tracer_path 
   * @param config_path 
   */
  TracedService(const std::string& tracer_path, const std::string& config_path)
  {
    initializeTracer(tracer_path, config_path);
    traced_service_ = nh_.advertiseService("service_1", &TracedService::tracingServiceCallback, this);
  }

  /**
   * @brief Destroy the Traced Service object
   * 
   */
  ~TracedService()
  {
    (*tracer_)->Close();
  }

private:
  int initializeTracer(const std::string& tracer_path, const std::string& config_path)
  {
    // Load the tracer library.
    std::string error_message;
    handle_maybe_ = opentracing::DynamicallyLoadTracingLibrary(tracer_path.c_str(), error_message);
    if (!handle_maybe_) 
    {
      ROS_ERROR_STREAM("Failed to load tracer library " << error_message);
      return -1;
    }

    // Read in the tracer's configuration.
    std::ifstream istream{config_path.c_str()};
    if (!istream.good()) 
    {
      ROS_ERROR_STREAM("Failed to open tracer config file " << config_path << ": "
                << std::strerror(errno));
      return -1;
    }

    std::string tracer_config{std::istreambuf_iterator<char>{istream},
                              std::istreambuf_iterator<char>{}};
    
    // Construct a tracer
    tracer_ = handle_maybe_->tracer_factory().MakeTracer(tracer_config.c_str(), error_message);

    if (!tracer_) 
    {
      ROS_ERROR_STREAM("Failed to create tracer " << error_message);
      return -1;
    }
  }

  bool tracingServiceCallback(temoto_core::Tracing::Request& req, temoto_core::Tracing::Response&)
  {
    temoto_core::StringMap string_map = temoto_core::keyValuesToUnorderedMap(req.context);
    TextMapCarrier carrier(string_map);
    auto span_context_maybe = (*tracer_)->Extract(carrier);
    assert(span_context_maybe);

    auto span = (*tracer_)->StartSpan("child_span", {ChildOf(span_context_maybe->get())});
    span->Log({{"info", "just chillin"}});
  }

  ros::NodeHandle nh_;
  ros::ServiceServer traced_service_;

  opentracing::v2::expected<opentracing::v2::DynamicTracingLibraryHandle> handle_maybe_;
  opentracing::v2::expected<std::shared_ptr<opentracing::v2::Tracer>> tracer_;
};

int main(int argc, char* argv[]) 
{
  /*
   * Set up ROS related stuff
   */ 
  ros::init(argc, argv, "child_tracer");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  TracedService traced_service(argv[1], argv[2]);

  ros::waitForShutdown();
  return 0;
}
