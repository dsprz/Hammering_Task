#include "BSpline.h"

#include "../HammeringFSM.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cmath>
#include <mc_rtc/gui/Button.h>

void BSpline::configure(const mc_rtc::Configuration & config)
{
}

void BSpline::start(mc_control::fsm::Controller & ctl_)
{
 
  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Stop hammering", [this]() { stop = true; }));

  Eigen::VectorXd dimW(6);
  dimW << 10, 10, 10, 1, 1, 1; 

  Eigen::VectorXd vel(6);
  vel << 0, 0, 0, 0, 0, 10; 

  const mc_trajectory::BSpline::waypoints_t & posWp ={Eigen::Vector3d({0.55, 0.30, 1.456}),
                                                      Eigen::Vector3d({0.55, 0.30, 0.90})};
  
  const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0.5, -0.5, -0.5, 0.5}), Eigen::Vector3d({0.55, 0.30, 0.90}));
  
  BSpline = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame("Hammer_Head"), 5,5.0, 1000, target, posWp);

  BSpline->stiffness(10);
  BSpline->weight(1000);

  BSpline->dimWeight(dimW);



  ctl.getPostureTask(ctl.robot().name())->weight(1);
  ctl.solver().addTask(BSpline);
}

bool BSpline::run(mc_control::fsm::Controller & ctl_)
{
    mc_rtc::log::info("BSpline");

    if( BSpline->eval().norm() < 0.02)
    {
        switch_target();
    }

    if(stop)
    {
        output("Stop");
        return true;
    }
  
  return false;
}

void BSpline::teardown(mc_control::fsm::Controller & ctl_)
{
  
    mc_rtc::log::info("teardown BSpline");

  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl_.gui()->removeElement({}, "Stop hammering");

  ctl.solver().removeTask(BSpline);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

void BSpline::switch_target()
{
   
  hammering=!hammering;
  if(hammering)
  {
    BSpline->spline().start(Eigen::Vector3d({0.55, 0.30, 1.456}));

    BSpline->target(sva::PTransformd(Eigen::Quaterniond({0.5, -0.5, -0.5, 0.5}), Eigen::Vector3d({0.55, 0.30, 0.90})));
  }
  else
  {
    BSpline->spline().start(Eigen::Vector3d({0.55, 0.30, 0.90}));

    BSpline->target(sva::PTransformd(Eigen::Quaterniond({-0.5, 0.5, 0.5, -0.5}), Eigen::Vector3d({0.55, 0.30, 1.456})));

  }
}
 
EXPORT_SINGLE_STATE("BSpline", BSpline)