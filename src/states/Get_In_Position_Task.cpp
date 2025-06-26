#include "Get_In_Position_Task.h"

#include "../Hammering_FSM_Controller.h"
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
#include <ostream>

void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
 
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  initial_hammer_head_position = ctl.robot().frame("Hammer_Head").position();


  ctl.gui()->addElement({}, mc_rtc::gui::Button("Stop hammering", [this]() { stop = true; }));

  constr.end_vel.z() = -1;
  constr.end_acc.z() = 0;
  constr.end_jerk.z() = 0;

  constr.end_vel.x() = 0;
  constr.end_vel.y() = 0;

  constr.end_acc.x() = 0;
  constr.end_acc.y() = 0;
  
  constr.end_jerk.x() = 0;
  constr.end_jerk.y() = 0;

  auto nail_pos = ctl.robots().robot("nail").frame("nail").position();
  double x_nail = nail_pos.translation().x();
  double y_nail = nail_pos.translation().y();

  double x = initial_hammer_head_position.translation().x();
  double y = initial_hammer_head_position.translation().y();
  double z = initial_hammer_head_position.translation().z();

  auto startPoint = initial_hammer_head_position.translation();
  auto endPoint = nail_pos.translation();

  // std::cout << "Matrix : " << rotation << std::endl;
  std::cout << z << std::endl;
  posWp ={initial_hammer_head_position.translation(),
          // Eigen::Vector3d({x, y, 1.3}),
          Eigen::Vector3d({0.55, 0.30,1.456}),
        nail_pos.translation(),                                    
          };
  
  Eigen::Quaterniond identity_quaternion = Eigen::Quaterniond({1, 0, 0, 0});
  
  std::cout << "Initial rotation matrix of the hammer head : " << std::endl;
  std::cout << initial_hammer_head_position.rotation() << std::endl;
  
  std::cout << "Quaternion rotation matrix : " << std::endl;
  std::cout << Eigen::Quaterniond({0, -0.7, 0, 0.7}).matrix() << std::endl;
  
  const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {
    std::make_pair(1, Eigen::Quaterniond({0, -0.7, 0, 0.7}).matrix())
  };//std::make_pair(4, Eigen::Quaterniond({0.51, -0.28, 0.24, 0.77}).matrix())};

  // const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), Eigen::Vector3d({0.55, 0.30, 0.9}));
  const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), Eigen::Vector3d({0.55, 0.30, 0.9}));
  
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame("Hammer_Head"),
                                                                  5, 
                                                                  5.0, 
                                                                  1000, 
                                                                  target, 
                                                                  constr, 
                                                                  posWp, 
                                                                  oriWp,
                                                                  false);
  BSplineVel->stiffness(100);
  BSplineVel->weight(1000);
  std::cout << "BSplineVel->spline().get_bezier().constr_.end_vel.z(): " << BSplineVel->spline().get_bezier()->constr_.end_vel.z() << std::endl;
  // std::cout << "end vel z : " << constr.end_vel.z() << std::endl;

  ctl.getPostureTask(ctl.robot().name())->weight(1);
  ctl.solver().addTask(BSplineVel);
}

bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    // auto vel = ctl.robot().frame("Hammer_Head").velocity();
    // auto linear_vel = vel.linear();
    // Eigen::Vector3d world_linear = ctl.robot().frame("Hammer_Head").position().rotation() * vel.linear();
    // std::cout << "linear_vel hammer_frame  = {" << std::endl;
    // std::cout << "linear_vel.x = " << linear_vel.x() << std::endl;
    // std::cout << "linear_vel.y = " << linear_vel.y() << std::endl;
    // std::cout << "linear_vel.z = " << linear_vel.z() << std::endl;
    // std::cout << "}";
    // std::cout << "" << std::endl;
    
    double epsilon = 0.13;
    if( BSplineVel->eval().norm() < epsilon)
    {
        output("Stop");
        std::cout << "Final rotation matrix of the hammer head : " << std::endl;
        std::cout << ctl.robot().frame("Hammer_Head").position().rotation() << std::endl;
        return true;
    }
    else {
      // std::cout << "output is not 'Stop' : "  << "BSplineVel->eval().norm() = " << BSplineVel->eval().norm() << " > epsilon = " << epsilon << std::endl;
    }

  
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

  ctl_.gui()->removeElement({}, "Stop hammering");

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}



EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)