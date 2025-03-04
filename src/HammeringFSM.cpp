#include "HammeringFSM.h"
#include <SpaceVecAlg/PTransform.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/Collision.h>

HammeringFSM::HammeringFSM(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::Tasks)
{
  robot().frame("Hammer_Head").velocity(); 
  
  mc_rtc::log::success("HammeringFSM init done ");

}

bool HammeringFSM::run()
{
  this->robot().frame("Hammer_Head").body();
  return mc_control::fsm::Controller::run();
}

void HammeringFSM::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


