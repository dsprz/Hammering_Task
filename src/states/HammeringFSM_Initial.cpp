#include "HammeringFSM_Initial.h"

#include "../HammeringFSM.h"

void HammeringFSM_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringFSM_Initial::start(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { hammering_ = true; }));
}

bool HammeringFSM_Initial::run(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("initial");
  if(hammering_)
  {
    output("Hammering");
    return true;
  }
  return false;
}

void HammeringFSM_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
   ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringFSM_Initial", HammeringFSM_Initial)
