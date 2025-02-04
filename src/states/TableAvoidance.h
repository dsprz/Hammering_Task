#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/TransformTask.h>

struct TableAvoidance : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private: 
    std::shared_ptr<mc_tasks::TransformTask> transformTask;

};