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
  yaml-cpp
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

set(headers
  include
  ${catkin_INCLUDE_DIRS}
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
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs message_generation yaml-cpp
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
)
target_link_libraries(temoto_core_components 
  ${catkin_LIBRARIES}
  ${libraries}
  libyaml-cpp.so
)

install(TARGETS temoto_core_components
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)