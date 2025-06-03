cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME Hammering_FSM_Controller)
set(PROJECT_DESCRIPTION "Hammering_FSM_Controller")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(plugin)

set(AROBASE "@")
configure_file(etc/Hammering_FSM_Controller.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/Hammering_FSM_Controller.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/Hammering_FSM_Controller.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

