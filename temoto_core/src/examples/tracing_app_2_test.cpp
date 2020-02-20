#include <opentracing/dynamic_load.h>
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <text_map_carrier.h>
#include <memory>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include "temoto_core/Tracing.h"
#include "temoto_core/common/tracer_conversions.h"

using namespace opentracing;

typedef std::shared_ptr<int> SharedInt;

void myRefTest(const SharedInt& data)
{
  std::cout << "Address of data is: " << &data << std::endl;
}

int main(int argc, char* argv[]) 
{
  YAML::Node config;
  int jama = 5;
  std::cout << "Address of jama is: " << &config << std::endl;
  SharedInt arv = std::make_shared<int>(jama);
  std::cout << "Address of arv is: " << &arv << std::endl;
  myRefTest(arv);

  /*
   * Set up ROS related stuff
   */ 
  ros::init(argc, argv, "parent_tracer");
  ros::NodeHandle nh;
  ros::ServiceClient tr_client = nh.serviceClient<temoto_core::Tracing>("service_1");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  /*
   * Set up the tracer
   */ 
  if (argc != 3) 
  {
    std::cerr << "Usage: <tracer_library> <tracer_config_file>\n";
    return -1;
  }

  // Load the tracer library.
  std::string error_message;
  auto handle_maybe = opentracing::DynamicallyLoadTracingLibrary(argv[1], error_message);
  if (!handle_maybe) 
  {
    std::cerr << "Failed to load tracer library " << error_message << "\n";
    return -1;
  }

  // Read in the tracer's configuration.
  std::ifstream istream{argv[2]};
  if (!istream.good()) 
  {
    std::cerr << "Failed to open tracer config file " << argv[2] << ": "
              << std::strerror(errno) << "\n";
    return -1;
  }

  // std::string tracer_config{std::istreambuf_iterator<char>{istream},
  //                           std::istreambuf_iterator<char>{}};

  std::string tracer_name = "liljohns";
  std::string tracer_config = std::string("service_name: " + tracer_name + "\n");
  std::string tracer_config_other{ 
    std::istreambuf_iterator<char>{istream}
  , std::istreambuf_iterator<char>{}};
  tracer_config += tracer_config_other;


  // Construct a tracer.
  auto& tracer_factory = handle_maybe->tracer_factory();
  auto tracer_maybe = tracer_factory.MakeTracer(tracer_config.c_str(), error_message);

  if (!tracer_maybe) 
  {
    std::cerr << "Failed to create tracer " << error_message << "\n";
    return -1;
  }
  auto& tracer = *tracer_maybe;

  /*
   * Create a span and propagate it to the child
   */ 
  auto parent_span = tracer->StartSpan("parent_span");
  // parent_span->SetTag("asking for", "trouble");
  // assert(parent_span);

  // // Get the tracker context
  // temoto_core::StringMap string_map;
  // TextMapCarrier carrier(string_map);
  // auto err = tracer->Inject(parent_span->context(), carrier);
  // assert(err);

  // // Convert it to a key value pair vector and send it to a child tracer
  // temoto_core::KeyValues key_values = temoto_core::unorderedMapToKeyValues(string_map);
  // temoto_core::Tracing tracing_srv_msg;
  // tracing_srv_msg.request.context = key_values;
  
  // // Call the child tracer server
  // if (!tr_client.call(tracing_srv_msg))
  // {
  //   ROS_ERROR_STREAM("Unable to call the service.");
  //   parent_span->Log({{"error", "Unable to call the service"}});
  // }

  parent_span->Finish();
  tracer->Close();

  return 0;
}
