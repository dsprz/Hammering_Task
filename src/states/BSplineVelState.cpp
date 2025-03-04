#include "BSplineVelState.h"

#include "../HammeringFSM.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cmath>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_trajectory/BSpline.h>

void BSplineVelState::configure(const mc_rtc::Configuration & config)
{
}

void BSplineVelState::start(mc_control::fsm::Controller & ctl_)
{
 
  auto & ctl = static_cast<HammeringFSM &>(ctl_);
  initial_pose_left = ctl.robot().frame("Hammer_Head").position();


  ctl.gui()->addElement({}, mc_rtc::gui::Button("Stop hammering", [this]() { stop = true; }));

  Eigen::VectorXd dimW(6);
  dimW << 1, 1, 0, 1, 1, 1; 

  constr.end_vel.z() = -1;
  constr.end_acc.z() = 5;

  double x = initial_pose_left.translation().x();
  double y = initial_pose_left.translation().y();
  const mc_trajectory::BSpline::waypoints_t & posWp ={ initial_pose_left.translation(),
                                                        Eigen::Vector3d({x, y, 1}),
                                                        Eigen::Vector3d({x, y, 1.1}),
                                                        Eigen::Vector3d({x, y, 1.2}),
                                                        Eigen::Vector3d({x, y, 1.3}),
                                                        Eigen::Vector3d({x,y,1.456}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.456}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.3}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.2}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.1}),
                                                      Eigen::Vector3d({0.55, 0.30, 1}),
                                                      Eigen::Vector3d({0.55, 0.30, 0.90})};

  const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), Eigen::Vector3d({0.55, 0.30, 0.90}));
  
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame("Hammer_Head"),5, 5.0, 1000, target, constr, posWp);

  BSplineVel->stiffness(10);
  BSplineVel->weight(1000);

  BSplineVel->dimWeight(dimW);

  ctl.getPostureTask(ctl.robot().name())->weight(1);
  ctl.solver().addTask(BSplineVel);
}

bool BSplineVelState::run(mc_control::fsm::Controller & ctl_)
{
    if( BSplineVel->eval().norm() < 0.02 or stop)
    {
        output("Stop");
        return true;
    }
  
  return false;
}

void BSplineVelState::teardown(mc_control::fsm::Controller & ctl_)
{
  
  mc_rtc::log::info("teardown BSpline");

  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl_.gui()->removeElement({}, "Stop hammering");

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}
 
EXPORT_SINGLE_STATE("BSplineVelState", BSplineVelState)