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

void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
 
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  initial_hammer_head_position = ctl.robot().frame("Hammer_Head").position();


  ctl.gui()->addElement({}, mc_rtc::gui::Button("Stop hammering", [this]() { stop = true; }));

  constr.end_vel.z() = -1;
  constr.end_acc.z() = 5;
  constr.end_jerk.z() = 0;

  auto nail_pos = ctl.robots().robot("nail").frame("nail").position();
  double x_nail = nail_pos.translation().x();
  double y_nail = nail_pos.translation().y();

  double x = initial_hammer_head_position.translation().x();
  double y = initial_hammer_head_position.translation().y();

  auto startPoint = initial_hammer_head_position.translation();
  auto endPoint = nail_pos.translation();

  // auto rotation = initial_hammer_head_position.rotation();

  // std::cout << "Matrix : " << rotation << std::endl;

  const mc_trajectory::BSpline::waypoints_t & posWp ={ initial_hammer_head_position.translation(),
                                                        Eigen::Vector3d({x, y, 1}),
                                                        Eigen::Vector3d({x, y, 1.1}),
                                                        Eigen::Vector3d({x, y, 1.2}),
                                                        Eigen::Vector3d({x, y, 1.3}),
                                                        Eigen::Vector3d({0.55, 0.30,1.456}),
                                                        
                                                        Eigen::Vector3d({0.55, 0.30, 1.3}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.2}),
                                                        Eigen::Vector3d({0.55, 0.30, 1.1}),
                                                        Eigen::Vector3d({0.55, 0.30, nail_pos.translation().z()}),
                                                      
                                                      // Eigen::Vector3d({0.55, 0.30, 1}),
                                                      // Eigen::Vector3d({0.55, 0.30, 1}),
                                                      // Eigen::Vector3d({0.55, 0.30, 1}),
                                                      // Eigen::Vector3d({0.55, 0.30, 1}),
                                                      // Eigen::Vector3d({0.55, 0.30, 1}),

                          
                                                      // Eigen::Vector3d({0.55, 0.30, 1.4}),
                                                      //tst
                                                      };


  const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {std::make_pair(1, Eigen::Quaterniond({0.51, -0.28, 0.24, 0.77}).matrix())};

  const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), Eigen::Vector3d({0.55, 0.30, 0.9}));
  
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame("Hammer_Head"),
                                                                  5, 
                                                                  5.0, 
                                                                  1000, 
                                                                  target, 
                                                                  constr, 
                                                                  posWp, 
                                                                  oriWp);

  BSplineVel->stiffness(100);
  BSplineVel->weight(1000);

  ctl.getPostureTask(ctl.robot().name())->weight(1);
  ctl.solver().addTask(BSplineVel);
}

bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

    auto startPoint = ctl.robot().frame("Hammer_Head").position();
    if (startPoint.translation().z() > 1){
    const mc_trajectory::BSpline::waypoints_t & posWp ={ startPoint.translation(),

                                                        Eigen::Vector3d({1, 0.30, 1.456}),
                                                        
                                                        Eigen::Vector3d({1, 0.30, 1.3}),
                                                        Eigen::Vector3d({1, 0.30, 1.2}),
                                                        Eigen::Vector3d({1, 0.30, 1.1}),
                                                        Eigen::Vector3d({1, 0.30, 0.9})};

      BSplineVel->posWaypoints(posWp);
    }
    if( BSplineVel->eval().norm() < 0.09)
    {
        output("Stop");
        return true;
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