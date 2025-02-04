#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>

#include "api.h"

struct HammeringFSM_DLLAPI HammeringFSM : public mc_control::fsm::Controller
{
  HammeringFSM(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

};