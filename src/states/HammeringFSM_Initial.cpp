#include "HammeringFSM_Initial.h"

#include "../HammeringFSM.h"
#include <Tasks/Tasks.h>
#include <mc_tasks/PostureTask.h>

void HammeringFSM_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringFSM_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);
  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() { hammering_ = true; }));
  ctl_.getPostureTask(ctl_.robot().name())->weight(1);

}

bool HammeringFSM_Initial::run(mc_control::fsm::Controller & ctl_)
{
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
