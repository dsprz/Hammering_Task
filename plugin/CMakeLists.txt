cmake_minimum_required(VERSION 3.1)

project(
    NailSensorPlugin
    LANGUAGES CXX
    VERSION 1.0.0)

set(CATKIN_INSTALL_INTO_PREFIX_ROOT FALSE)

if(NOT TARGET mc_rtc::mc_control)
    find_package(mc_rtc REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

configure_file(etc/NailSensorPlugin.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/NailSensorPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/NailSensorPlugin.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")
        
set(plugin_SRC src/NailSensorPlugin.cpp)
set(plugin_HDR src/NailSensorPlugin.h)

add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_rtc_ros ${catkin_LIBRARIES})


