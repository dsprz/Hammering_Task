#pragma once

#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/EigenUtility.h>
#include <SpaceVecAlg/fwd.h>
#include <mc_control/fsm/State.h>

#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/api.h>
#include <mc_trajectory/BSpline.h>
#include <mc_trajectory/api.h>

#include <ndcurves/curve_constraint.h>


struct HammeringFSM_Hammering : mc_control::fsm::State
{

  typedef Eigen::Vector3d Point;
  typedef Point point_t;
  typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
  
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private: 
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> BSplineVel;
    sva::PTransformd initial_pose_left;
    curve_constraints_t constr;

    bool hammering = false;
    bool stop = false;
};