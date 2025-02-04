#include "TableAvoidance.h"

#include "../HammeringFSM.h"
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

void TableAvoidance::configure(const mc_rtc::Configuration & config)
{
}

void TableAvoidance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  transformTask = std::make_shared<mc_tasks::TransformTask>(ctl.robot("hammer").frame("hammer_head"), 10, 1000);
  // transformTask->selectActiveJoints({"LSC", "LSP", "LSR","LSY", "LEP", "LWRY", "LWRR", "LWRP"});
  
  ctl.solver().addTask(transformTask);

  ctl.gui()->addElement({}, mc_rtc::gui::Button("Stop Table Avoidance", [&]{output("OK");}));
  ctl.getPostureTask(ctl.robot().name())->weight(1);
}

bool TableAvoidance::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);
  mc_rtc::log::info("table avoidance");
  
  if(!output().empty())
    return true;

  return false;
}

void TableAvoidance::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl.solver().removeTask(transformTask);
  ctl.gui()->removeElement({}, "Stop Table Avoidance");
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}
 
EXPORT_SINGLE_STATE("TableAvoidance", TableAvoidance)