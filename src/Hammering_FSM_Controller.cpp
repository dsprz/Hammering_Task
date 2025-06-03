#include "Hammering_FSM_Controller.h"
#include <mc_rbdyn/RobotLoader.h>

Hammering_FSM_Controller::Hammering_FSM_Controller(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  datastore().make<std::string>("ControlMode", "Torque");
  datastore().make<std::string>("Coriolis", "Yes"); 

    mc_rtc::log::success("Hammering_FSM_Controller init done ");
}

bool Hammering_FSM_Controller::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);

}

void Hammering_FSM_Controller::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


