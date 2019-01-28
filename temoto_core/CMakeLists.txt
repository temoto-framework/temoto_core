cmake_minimum_required(VERSION 2.8.3)
project(temoto_core)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  genmsg
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
  diagnostic_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES temoto_core
  CATKIN_DEPENDS std_msgs diagnostic_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# # # # # # # # # # # #
# TEMOTO CORE
# # # # # # # # # # # #
add_library(temoto_core_components 
  src/common/reliability.cpp
  src/temoto_error/temoto_error.cpp
)
add_dependencies(temoto_core_components ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_core_components ${catkin_LIBRARIES})
