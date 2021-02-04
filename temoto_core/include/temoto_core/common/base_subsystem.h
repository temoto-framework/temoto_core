/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2020 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Author: Robert Valner */

#ifndef TEMOTO_CORE__BASE_SUBSYSTEM_H
#define TEMOTO_CORE__BASE_SUBSYSTEM_H

#include "temoto_core/temoto_error/temoto_error.h"
#include "temoto_core/common/tracer_conversions.h"
#include <string>

#ifdef enable_tracing
  #include <opentracing/dynamic_load.h>
  #include <text_map_carrier.h>
  #include <fstream>
  #include "ros/package.h"
  #include "yaml-cpp/yaml.h"
  #define TRACER (*(this->tracer_))
#endif

namespace temoto_core
{
class BaseSubsystem
{
public:

  /**
   * @brief Default constructor
   * 
   */
  BaseSubsystem() = default;

  /**
   * @brief Copy constructor
   * 
   * @param b 
   */
  BaseSubsystem(const BaseSubsystem& b) 
  : BaseSubsystem(b, b.class_name_)
  {}

  /**
   * @brief Assignment operator
   * 
   * @param b 
   * @return BaseSubsystem& 
   */
  BaseSubsystem& operator = (const BaseSubsystem& b)
  {
    BaseSubsystem(b, b.class_name_);
  }

  /**
   * @brief Copy constructor with class name
   * 
   * @param b 
   * @param class_name 
   */
  BaseSubsystem( const BaseSubsystem& b
               , const std::string& class_name)
  : subsystem_name_(b.subsystem_name_)
  , subsystem_code_(b.subsystem_code_)
  , class_name_(class_name)
  , log_group_(b.log_group_)
  , error_handler_(error::ErrorHandler(b.subsystem_code_, b.log_group_))
  {
    #ifdef enable_tracing
    initTracerWrapped();
    #endif
  }

  /**
   * @brief Constructor without log group
   * 
   * @param subsystem_name 
   * @param subsystem_code 
   * @param class_name 
   */
  BaseSubsystem( const std::string& subsystem_name
               , const error::Subsystem& subsystem_code
               , const std::string& class_name)
  : BaseSubsystem(subsystem_name, subsystem_code, class_name, subsystem_name)
  {}

  /**
   * @brief Constructor with all arguments
   * 
   * @param subsystem_name 
   * @param subsystem_code 
   * @param class_name 
   * @param log_group 
   */
  BaseSubsystem( const std::string& subsystem_name
               , const error::Subsystem& subsystem_code
               , const std::string& class_name
               , const std::string& log_group)
  : subsystem_name_(subsystem_name)
  , subsystem_code_(subsystem_code)
  , class_name_(class_name)
  , log_group_(log_group)
  , error_handler_(error::ErrorHandler(subsystem_code, log_group))
  {
    #ifdef enable_tracing
    initTracerWrapped();
    #endif
  }

  std::string subsystem_name_;
  error::Subsystem subsystem_code_;
  std::string class_name_;
  std::string log_group_;
  error::ErrorHandler error_handler_;

protected:
  /**
   * @brief This function is used when the BaseSubsystem cannot be initialized
   * during the construction phase
   * @param b
   */
  void initializeBase(const BaseSubsystem& b)
  {
    subsystem_name_ = b.subsystem_name_;
    subsystem_code_ = b.subsystem_code_;
    log_group_ = b.log_group_;
    error_handler_ = error::ErrorHandler(subsystem_code_, log_group_);

    #ifdef enable_tracing
    initTracerWrapped();
    #endif
  }

  /**
   * @brief Creates a string which contains information about the subsystem, class and name of the function
   * 
   * @param func_name 
   * @return std::string 
   */
  std::string generateLogPrefix(const std::string& func_name)
  {
    return common::generateLogPrefix(subsystem_name_, class_name_, func_name);
  }

#ifdef enable_tracing
  opentracing::v2::expected<opentracing::v2::DynamicTracingLibraryHandle> tracer_handle_maybe_;
  opentracing::v2::expected<std::shared_ptr<opentracing::v2::Tracer>> tracer_;
#endif

private:
#ifdef enable_tracing
  void initializeTracer( const std::string& tracer_config_path
                       , const std::string& tracer_name
                       , const std::string& tracer_lib_path = "")
  {
    TEMOTO_DEBUG_STREAM("Initializing the tracer");

    /*
     * Read in the tracer's configuration.
     */ 
    std::ifstream istream{tracer_config_path.c_str()};
    if (!istream.good()) 
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL, "Failed to open tracer config file: " + tracer_config_path);
    }

    std::string tracer_config = std::string("service_name: " + tracer_name + "\n");
    std::string tracer_config_other{ 
      std::istreambuf_iterator<char>{istream}
    , std::istreambuf_iterator<char>{}};
    tracer_config += tracer_config_other;

    /*
     * Load the tracer library
     */ 
    std::string tracer_lib_str;
    if (tracer_lib_path.empty())
    {
      try
      {
        YAML::Node tracer_config_yaml = YAML::LoadFile(tracer_config_path);
        tracer_lib_str = tracer_config_yaml["library_path"].as<std::string>();
      }
      catch(...)
      {
        throw CREATE_ERROR(error::Code::RMP_FAIL, "Failed to parse tracer lib path from config: " + tracer_config_path);
      }
    }
    else
    {
      tracer_lib_str = tracer_lib_path;
    }

    std::string error_message;
    tracer_handle_maybe_ = opentracing::DynamicallyLoadTracingLibrary(tracer_lib_str.c_str(), error_message);
    if (!tracer_handle_maybe_) 
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL, "Failed to load tracer library: " + error_message);
    }

    /*
     * Construct a tracer
     */ 
    tracer_ = tracer_handle_maybe_->tracer_factory().MakeTracer(tracer_config.c_str(), error_message);

    if (!tracer_) 
    {
      throw CREATE_ERROR(error::Code::RMP_FAIL, "Failed to create a tracer: " + error_message);
    }
    TEMOTO_DEBUG_STREAM("Tracer initialized");
  }

  void initTracerWrapped()
  {
    try
    {
      // TODO: The lib path should be embedded in tracer config
      std::string base_path = ros::package::getPath("temoto_core");
      std::string tracer_config_path = base_path + "/config/tracer_config.yaml";
      std::string tracer_name = temoto_core::common::getTemotoNamespace() + "/" + subsystem_name_;
      initializeTracer(tracer_config_path, tracer_name);
    }
    catch (error::ErrorStack& error_stack)
    {
      SEND_ERROR(error_stack);
    }
    catch(...)
    {
      TEMOTO_ERROR_STREAM("Unable to initialize the tracer");
    }
  }
#endif
};

} // temoto_core namespace
#endif
