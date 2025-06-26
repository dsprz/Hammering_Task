#include "Hammering_FSM_Controller_Initial.h"
#include <mc_rtc/logging.h>

#include "../Hammering_FSM_Controller.h"

void Hammering_FSM_Controller_Initial::configure(const mc_rtc::Configuration & config)
{
}

void Hammering_FSM_Controller_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

  // Creates a button to start the movement
  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { _positionning_hammer_clicked = true; }));
  ctl_.getPostureTask(ctl_.robot().name())->weight(1);
  mc_rtc::log::info("Starting Initial State");

}

bool Hammering_FSM_Controller_Initial::run(mc_control::fsm::Controller & ctl_)
{
  // Outputs OK when the button "Start Hammering" is clicked
  // auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  
  if (_positionning_hammer_clicked)
  {
      output("BUTTON_CLICKED");
      return true;
  }
  return false;
}

void Hammering_FSM_Controller_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  // auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  ctl_.gui()->removeElement({}, "Start hammering");

}

EXPORT_SINGLE_STATE("Hammering_FSM_Controller_Initial", Hammering_FSM_Controller_Initial)
