#include "HammeringFSM.h"
#include <SpaceVecAlg/PTransform.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/Collision.h>

HammeringFSM::HammeringFSM(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::Tasks), ctlTime_(0)
{
  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("Coriolis", "Yes"); 
  
  mc_rtc::log::success("HammeringFSM init done ");

}

bool HammeringFSM::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void HammeringFSM::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


