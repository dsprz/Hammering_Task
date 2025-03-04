# pragma  once

#include <SpaceVecAlg/EigenTypedef.h>
#include <mc_control/GlobalPlugin.h>
#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/ros.h>
#include <mc_rbdyn/BodySensor.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <thread>
#include <vector>

namespace mc_plugin
{
    struct NailSensorPlugin : public mc_control::GlobalPlugin
    {
        void init(mc_control::MCGlobalController & controller,const mc_rtc::Configuration & config) override;

        void reset(mc_control::MCGlobalController & controller) override;

        void before(mc_control::MCGlobalController &) override;

        void after(mc_control::MCGlobalController & controller) override;

        mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

        ~NailSensorPlugin() override;

        private:
            ros::Subscriber _subForce;

            mc_rbdyn::BodySensor _nailBodySensor;

            void callback(const  geometry_msgs::Vector3StampedConstPtr &force);

            Eigen::Vector3d _force;

            std::thread _ros_thread;

    };
}