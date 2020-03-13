cmake_minimum_required(VERSION 2.8.3)
project(temoto_core)
add_compile_options(-std=c++11 -ggdb)

option(TEMOTO_ENABLE_TRACING "Build tracer related dependencies" OFF)

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  diagnostic_msgs
  message_generation
)

add_message_files(FILES 
  Error.msg
  ErrorStack.msg

  # Resource Management
  RMPRequest.msg
  RMPResponse.msg

  # Configuration Synchronizer
  ConfigSync.msg
)

add_service_files(
  FILES
  # Resouce Management
  UnloadResource.srv
  ResourceStatus.srv
  Tracing.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  diagnostic_msgs
)

# Build yaml-cpp version 0.6.2
include(ExternalProject)

ExternalProject_Add(yaml-cpp-062
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DYAML_CPP_BUILD_TESTS=OFF
    -Wno-dev
  BUILD_COMMAND $(MAKE) -j
  INSTALL_COMMAND cmake -E echo "Skipping install step."
)

add_custom_command(
  TARGET yaml-cpp-062 POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build/libyaml-cpp.so
    ${CATKIN_DEVEL_PREFIX}/lib/libyaml-cpp.so
)

set(headers
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include
)
set(libraries
  ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build/libyaml-cpp.so.0.6.2
)
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

  add_custom_command(
    TARGET open-tracing-150 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/output/libopentracing.so
      ${CATKIN_DEVEL_PREFIX}/lib/libopentracing.so
  )
  list(APPEND headers
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/example/tutorial
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/3rd_party/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/include
  )
  list(APPEND libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/build/output/libopentracing.so
    -pthread
  )
  list(APPEND src_files
    src/common/tracer_conversions.cpp
  )
  list(APPEND tracing_dependencies
    open-tracing-150
  )
endif()

catkin_package(
  INCLUDE_DIRS ${headers}
  LIBRARIES temoto_core_components ${libraries}
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs message_generation
  # DEPENDS
)

include_directories(${headers})

# # # # # # # # # # # #
# TEMOTO CORE COMPONENTS
# # # # # # # # # # # #
add_library(temoto_core_components 
  src/common/reliability.cpp
  src/temoto_error/temoto_error.cpp
  ${src_files}
)
add_dependencies(temoto_core_components
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${tracing_dependencies}
  yaml-cpp-062
)
target_link_libraries(temoto_core_components 
  ${catkin_LIBRARIES}
  ${libraries}
)

install(TARGETS temoto_core_components
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)