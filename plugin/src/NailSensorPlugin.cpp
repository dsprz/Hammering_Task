#include "NailSensorPlugin.h"

#include <boost/math/special_functions/math_fwd.hpp>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc/logging.h>

#include <ros/init.h>
#include <ros/time.h>
#include <ros/topic.h>

namespace mc_plugin
{
    NailSensorPlugin::~NailSensorPlugin() = default;


    void mc_plugin::NailSensorPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
    {
        mc_rtc::log::info("NailSensorPlugin::init called with configuration: \n{}", config.dump(true, true));
        auto nh = mc_rtc::ROSBridge::get_node_handle();

        if(nh != nullptr){
            _subForce = nh->subscribe("/nail_force_sensor", 1000, &NailSensorPlugin::callback, this);
            _ros_thread = std::thread([]{while (ros::ok()){
                ros::spinOnce();
            }});

            mc_rtc::log::info("NailSensorPlugin::init waiting for force on nail head on topic /nail_force_sensor");
        }else
            mc_rtc::log::error("Run roscore before running NailSensorPlugin");

        auto sensor_config = config.find(controller.robot().module().name);

        if(sensor_config)
        {
            std::string name = (*sensor_config)("sensorName");
            std::string bodyname = (*sensor_config)("sensorBody");
            sva::PTransformd transform = (*sensor_config)("sensorTransform");

            _nailBodySensor = mc_rbdyn::BodySensor(name, bodyname, transform);

            controller.controller().robot("nail").addBodySensor(_nailBodySensor);
        }

        controller.controller().logger().addLogEntry("nail_force_sensor", [this](){return _force;});

        mc_rtc::log::success("NailSensorPlugin Running");
    }

    void NailSensorPlugin::reset(mc_control::MCGlobalController &controller)
    {
        mc_rtc::log::info("NailSensorPlugin::reset called");
    }

    void NailSensorPlugin::before(mc_control::MCGlobalController &controller)
    {
    }

    void NailSensorPlugin::after(mc_control::MCGlobalController &controller)
    {
    }

    void NailSensorPlugin::callback(const geometry_msgs::Vector3StampedConstPtr &force){
        _force = {force->vector.x, force->vector.y, force->vector.z};
    }

    mc_control::GlobalPlugin::GlobalPluginConfiguration NailSensorPlugin::configuration()
    {
        mc_control::GlobalPlugin::GlobalPluginConfiguration out;
        out.should_run_before = true;
        out.should_run_after = true;
        out.should_always_run = true;
        return out;
    }

}

EXPORT_MC_RTC_PLUGIN("NailSensorPlugin", mc_plugin::NailSensorPlugin)