cmake_minimum_required(VERSION 2.8.3)
project(temoto_core)
add_compile_options(-std=c++11 -ggdb)

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

ExternalProject_Add(yaml-cpp062
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS=ON
    -DYAML_CPP_BUILD_TESTS=OFF
    -Wno-dev
  BUILD_COMMAND $(MAKE) -j
  INSTALL_COMMAND cmake -E echo "Skipping install step."
)

set(headers
  include
  ${catkin_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/include"
)
set(libraries
  "${CMAKE_CURRENT_SOURCE_DIR}/yaml-cpp/build/libyaml-cpp.so"
)
set(src_files)

if( "$ENV{TEMOTO_ENABLE_TRACING}" STREQUAL "true")
  add_compile_options(-Denable_tracing)
  list(APPEND headers
    "${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/example/tutorial"
    "${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/3rd_party/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/.build/include"
  )
  list(APPEND libraries
    "${CMAKE_CURRENT_SOURCE_DIR}/tracing/opentracing-cpp/.build/output/libopentracing.so"
    "-pthread"
  )
  list(APPEND src_files
    src/common/tracer_conversions.cpp
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
  yaml-cpp062
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
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# TESTING
# add_executable(tracing_app_1_test
#   src/tracing_app_1_test.cpp
#   ${src_files}
# )
# add_dependencies(tracing_app_1_test 
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# target_link_libraries(tracing_app_1_test
#   ${libraries}
# )

# add_executable(tracing_app_2_test
#   src/tracing_app_2_test.cpp
#   ${src_files}
# )
# add_dependencies(tracing_app_2_test 
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   yaml-cpp062
# )
# target_link_libraries(tracing_app_2_test
#   ${catkin_LIBRARIES}
#   ${libraries}
# )

# add_executable(tracing_app_3_test
#   src/tracing_app_3_test.cpp
#   ${src_files}
# )
# add_dependencies(tracing_app_3_test 
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# target_link_libraries(tracing_app_3_test
#   ${catkin_LIBRARIES}
#   ${libraries}
# )