#include "Hammering_FSM_Controller.h"
#include <mc_rbdyn/RobotLoader.h>

Hammering_FSM_Controller::Hammering_FSM_Controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  datastore().make<std::string>("ControlMode", "Torque");

  // Here you can change the robot module name with your own robot
  // for(const auto &f : robot().frames()){
  //   mc_rtc::log::info("frame : {}", f);
  // }
  const auto & jointNames = robot().module().ref_joint_order();
  // for(const auto &j : jointNames){
    //    std::cout << "joint order : " << j << std::endl;
    // }
    // Print each joint name with its corresponding value
    for (size_t i = 0; i < jointNames.size(); ++i) 
    {
      std::cout << jointNames[i] << ": " << "[";
      for (auto &v : robot().q()[i])
      {
        std::cout << v << ", ";
      }
      std::cout << "]" << std::endl;
    }
    mc_rtc::log::success("Hammering_FSM_Controller init done ");
}

bool Hammering_FSM_Controller::run()
{
  return mc_control::fsm::Controller::run();
}

void Hammering_FSM_Controller::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


