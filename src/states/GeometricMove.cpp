#include "GeometricMove.h"

#include "../HammeringFSM.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <cmath>
#include <mc_rtc/gui/Button.h>

void GeometricMove::configure(const mc_rtc::Configuration & config)
{
}

void GeometricMove::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl_.gui()->addElement({}, mc_rtc::gui::Button("Stop hammering", [this]() { stop = true; }));


  initial_pose_left = ctl.robot().frame("Hammer_Head").position();
  sva::MotionVecd velocity(Eigen::Vector6d::Zero()); 

  transformTaskLeft = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame("Hammer_Head"), 10, 100000);
  transformTaskLeft->target(sva::PTransformd(initial_pose_left.rotation(), {0.55, 0.30, 1.456}));

  ctl.solver().addTask(transformTaskLeft);

}

bool GeometricMove::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);
  mc_rtc::log::info("geometric move");
  
  if(transformTaskLeft->eval().norm() < 1e-2){
    output("OK");
    return true;
    // switch_target();
  }

  if(stop)
  {
      output("Stop");
      return true;
  }

  return false;
}

void GeometricMove::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringFSM &>(ctl_);

  ctl.solver().removeTask(transformTaskLeft);
  ctl_.gui()->removeElement({}, "Stop hammering");
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

// void GeometricMove::switch_target()
// {
//   Eigen::Vector6d vel;
//   hammering=!hammering;
//   if(hammering)
//   {
//     vel << 0, 0, 0, 0, 0, 1;
//     transformTaskLeft->target(sva::PTransformd(Eigen::Quaterniond({0.5, -0.5, -0.5, 0.5}), {0.55, 0.30, 0.90}));

//   }
//   else
//   {
//     vel << 0, 0, 0, 0, 0, 0;
//     transformTaskLeft->target(sva::PTransformd(Eigen::Quaterniond({0.5, -0.5, -0.5, 0.5}), {0.55, 0.30, 1.456}));
//   }

//   // transformTaskLeft->targetVel(vel);
// }
 
EXPORT_SINGLE_STATE("GeometricMove", GeometricMove)