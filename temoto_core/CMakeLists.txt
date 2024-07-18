cmake_minimum_required(VERSION 3.8)
project(temoto_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++11 -ggdb)
option(TEMOTO_ENABLE_TRACING "Build tracer related dependencies" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization program_options)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Error.msg"
  "msg/ErrorStack.msg"
  "msg/RMPRequest.msg"
  "msg/RMPResponse.msg"
  "msg/ConfigSync.msg"
  "srv/UnloadResource.srv"
  "srv/ResourceStatus.srv"
  "srv/Tracing.srv"
  
  DEPENDENCIES std_msgs diagnostic_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${headers}
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_core/include
)
set(libraries)
set(src_files)

set(tracing_dependencies)

if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)

  # Build OpenTracing 1.5.0
  include(ExternalProject)

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/include)

  ExternalProject_Add(open-tracing-150
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp
    BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build
    BUILD_COMMAND $(MAKE) -j
    INSTALL_COMMAND cmake -E echo "Skipping install step."
  )

  list(APPEND headers
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/example/tutorial
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/3rd_party/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/include
  )
  list(APPEND libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/output/libopentracing.so
    pthread
  )
  list(APPEND src_files
    src/common/tracer_conversions.cpp
  )
  list(APPEND tracing_dependencies
    open-tracing-150
  )
endif()

# # # # # # # # # # # #
# TEMOTO CORE COMPONENTS
# # # # # # # # # # # #
add_library(temoto_core_components 
  src/common/reliability.cpp
  src/temoto_error/temoto_error.cpp
  ${src_files}
)
target_include_directories(temoto_core_components PUBLIC
  ${headers}
  ${rclcpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_core/include
)
ament_target_dependencies(temoto_core_components
  rclcpp
  yaml-cpp
  Boost
)
target_link_libraries(temoto_core_components
  
)

rosidl_target_interfaces(temoto_core_components
  ${PROJECT_NAME} "rosidl_typesupport_cpp")



# Install library
install(TARGETS temoto_core_components
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


# Install .srv files
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME})

# Install .msg files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}  
)



# Export information to downstream packages
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()