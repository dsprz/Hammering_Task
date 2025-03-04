#pragma once

#include <SpaceVecAlg/fwd.h>
#include <mc_control/fsm/State.h>

#include <mc_tasks/TransformTask.h>

struct GeometricMove : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private: 
    std::shared_ptr<mc_tasks::TransformTask> transformTaskLeft;
    sva::PTransformd initial_pose_left;

    bool hammering = false;
    bool stop = false;

};