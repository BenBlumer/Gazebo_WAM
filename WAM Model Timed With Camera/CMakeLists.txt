cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(set_joints_timed_cam SHARED plugins/set_joints_timed_cam.cc)
target_link_libraries(set_joints_timed_cam ${GAZEBO_libraries})

add_library(model_push SHARED plugins/model_push.cc)
target_link_libraries(model_push ${GAZEBO_libraries})
