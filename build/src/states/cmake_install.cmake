# Install script for directory: /home/helene/src/hammering_fsm/src/states

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/helene/workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so"
         RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states" TYPE SHARED_LIBRARY FILES "/home/helene/src/hammering_fsm/build/src/states/HammeringFSM_Hammering.so")
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so"
         OLD_RPATH "/home/helene/src/hammering_fsm/build/src:/home/helene/workspace/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Hammering.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so"
         RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states" TYPE SHARED_LIBRARY FILES "/home/helene/src/hammering_fsm/build/src/states/HammeringFSM_Initial.so")
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so"
         OLD_RPATH "/home/helene/src/hammering_fsm/build/src:/home/helene/workspace/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Initial.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so"
         RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states" TYPE SHARED_LIBRARY FILES "/home/helene/src/hammering_fsm/build/src/states/HammeringFSM_Finish.so")
  if(EXISTS "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so"
         OLD_RPATH "/home/helene/src/hammering_fsm/build/src:/home/helene/workspace/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/helene/workspace/install/lib/mc_controller/fsm/states:/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/HammeringFSM_Finish.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states/data")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/helene/workspace/install/lib/mc_controller/HammeringFSM/states" TYPE DIRECTORY FILES "/home/helene/src/hammering_fsm/src/states/data" FILES_MATCHING REGEX ".*(.json$|.yml$|.yaml$)")
endif()

