# Install script for directory: /home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states

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
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so"
         RPATH "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/fsm/states:/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states" TYPE SHARED_LIBRARY FILES "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states/Hammering_FSM_Controller_Initial.so")
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so"
         OLD_RPATH "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src:/home/jimmyvu/Documents/mc_rtc/install/lib::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/fsm/states:/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Hammering_FSM_Controller_Initial.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so"
         RPATH "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/fsm/states:/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states" TYPE SHARED_LIBRARY FILES "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states/Get_In_Position_Task.so")
  if(EXISTS "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so"
         OLD_RPATH "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src:/home/jimmyvu/Documents/mc_rtc/install/lib::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/fsm/states:/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/Get_In_Position_Task.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states/data")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/jimmyvu/Documents/mc_rtc/install/lib/mc_controller/Hammering_FSM_Controller/states" TYPE DIRECTORY FILES "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states/data" FILES_MATCHING REGEX ".*(.json$|.yml$|.yaml$)")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/jimmyvu/Documents/mc_rtc/sandbox/hammering_fsm_controller/src/states/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
