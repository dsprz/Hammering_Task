# Install script for directory: /home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/jimmyvu/Documents/mc_rtc/install/lib" TYPE SHARED_LIBRARY FILES "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/libHammering_FSM_Controller.so")
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so"
         OLD_RPATH "/home/jimmyvu/Documents/mc_rtc/install/lib:/home/jimmyvu/Documents/ros2_humble/install/nav_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/sensor_msgs/lib:/home/jimmyvu/Documents/mc_rtc/src/catkin_data_ws/install/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2_ros/lib:/home/jimmyvu/Documents/ros2_humble/install/message_filters/lib:/home/jimmyvu/Documents/ros2_humble/install/rclcpp_action/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_action/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/geometry_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/std_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/action_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/unique_identifier_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/rosbag2_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rclcpp/lib:/home/jimmyvu/Documents/ros2_humble/install/libstatistics_collector/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_logging_spdlog/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_logging_interface/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_interfaces/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_yaml_param_parser/lib:/home/jimmyvu/Documents/ros2_humble/install/libyaml_vendor/lib:/home/jimmyvu/Documents/ros2_humble/install/rosgraph_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/statistics_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/builtin_interfaces/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/fastcdr/lib:/home/jimmyvu/Documents/ros2_humble/install/tracetools/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rmw_implementation/lib:/home/jimmyvu/Documents/ros2_humble/install/rmw/lib:/home/jimmyvu/Documents/ros2_humble/install/rosbag2_storage/lib:/home/jimmyvu/Documents/ros2_humble/install/ament_index_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/class_loader/lib:/home/jimmyvu/Documents/ros2_humble/install/rcpputils/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_runtime_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rcutils/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_introspection_c/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/libHammering_FSM_Controller.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller" TYPE SHARED_LIBRARY FILES "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/Hammering_FSM_Controller_controller.so")
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so"
         OLD_RPATH "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src:/home/jimmyvu/Documents/mc_rtc/install/lib:/home/jimmyvu/Documents/ros2_humble/install/nav_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/sensor_msgs/lib:/home/jimmyvu/Documents/mc_rtc/src/catkin_data_ws/install/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2_ros/lib:/home/jimmyvu/Documents/ros2_humble/install/message_filters/lib:/home/jimmyvu/Documents/ros2_humble/install/rclcpp_action/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_action/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2/lib:/home/jimmyvu/Documents/ros2_humble/install/tf2_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/geometry_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/std_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/action_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/unique_identifier_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/rosbag2_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rclcpp/lib:/home/jimmyvu/Documents/ros2_humble/install/libstatistics_collector/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_logging_spdlog/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_logging_interface/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_interfaces/lib:/home/jimmyvu/Documents/ros2_humble/install/rcl_yaml_param_parser/lib:/home/jimmyvu/Documents/ros2_humble/install/libyaml_vendor/lib:/home/jimmyvu/Documents/ros2_humble/install/rosgraph_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/statistics_msgs/lib:/home/jimmyvu/Documents/ros2_humble/install/builtin_interfaces/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/fastcdr/lib:/home/jimmyvu/Documents/ros2_humble/install/tracetools/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rmw_implementation/lib:/home/jimmyvu/Documents/ros2_humble/install/rmw/lib:/home/jimmyvu/Documents/ros2_humble/install/rosbag2_storage/lib:/home/jimmyvu/Documents/ros2_humble/install/ament_index_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/class_loader/lib:/home/jimmyvu/Documents/ros2_humble/install/rcpputils/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_runtime_c/lib:/home/jimmyvu/Documents/ros2_humble/install/rcutils/lib:/home/jimmyvu/Documents/ros2_humble/install/rosidl_typesupport_introspection_c/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller_controller.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
