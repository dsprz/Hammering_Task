#include "Hammering_FSM_Controller_Initial.h"

#include "../Hammering_FSM_Controller.h"

void Hammering_FSM_Controller_Initial::configure(const mc_rtc::Configuration & config)
{
}

void Hammering_FSM_Controller_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
}

bool Hammering_FSM_Controller_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  output("OK");
  return true;
}

void Hammering_FSM_Controller_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
}

EXPORT_SINGLE_STATE("Hammering_FSM_Controller_Initial", Hammering_FSM_Controller_Initial)
