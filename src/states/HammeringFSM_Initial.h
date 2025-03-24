#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/PostureTask.h>

struct HammeringFSM_Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:
  std::shared_ptr<mc_tasks::PostureTask> PostureTask;
  bool hammering_ = false;
};
