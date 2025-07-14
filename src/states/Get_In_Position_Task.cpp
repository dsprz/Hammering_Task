#include "Get_In_Position_Task.h"

#include "../Hammering_FSM_Controller.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
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
#include <string>


void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  load_parameters();
  
  // Add a stop button to the gui
  ctl.gui()->addElement({}, mc_rtc::gui::Button(_stop_hammering_button_name, [this]() { stop = true; }));
  
  
  // Get the initial conditions
  _initial_hammerhead_position = ctl.robot().frame(_hammer_head_frame_name).position();
  auto initial_hammerhead_translation = _initial_hammerhead_position.translation();

  _initial_nail_position = ctl.robots().robot(_nail_robot_name).frame(_nail_frame_name).position();
  auto initial_nail_translation = _initial_nail_position.translation();

  double x_nail_init = initial_nail_translation.x();
  double y_nail_init = initial_nail_translation.y();
  double z_nail_init = initial_nail_translation.z();
  double x_hammer_init = initial_hammerhead_translation.x();
  double y_hammer_init = initial_hammerhead_translation.y();
  double z_hammer_init = initial_hammerhead_translation.z();

  _start_point = initial_hammerhead_translation;
  _end_point = initial_nail_translation;

  auto hammer_rot = _initial_hammerhead_position.rotation();
  auto nail_rot = _initial_nail_position.rotation();
  // std::cout << "nail rotation matrix : \n" << nail_rot << std::endl;


  // auto angle_with_trace = std::acos((nail_rot.trace()-1)/2);
  // std::cout << "angle with trace = " << angle_with_trace << std::endl;


  // Found by calculations by hand
  rotation_axis_ = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
  auto angle = M_PI;
  // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
  // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, rotation_axis_));

  posWp ={_start_point,
          Eigen::Vector3d({x_nail_init, y_nail_init, _magic_max_control_point_height}), //arbitrary z for now
        _end_point,                                    
          };
  
  
  const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {
    std::make_pair(_magic_oriWp_time, q.matrix()) // at t = _magic_oriWp_time, arbitrary for now

  };//std::make_pair(4, Eigen::Quaterniond({0.51, -0.28, 0.24, 0.77}).matrix())};

  // const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), nail_pos.translation());
  const sva::PTransformd & target = sva::PTransformd(q, //maybe the quaternion expresses the orientation of your rotating frame you want to achieve at the end with respect to the world frame
                                                            //thus whatever the starting orientation of the rotating frame wrt the world frame, the robot frame will try to end up at the orientation specified by the quaternion
                                                            //how does it do that ?
                                                  initial_nail_translation);
  
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(_hammer_head_frame_name),
                                                                  _magic_bezier_curve_max_duration, 
                                                                  _magic_task_stiffness, 
                                                                  _magic_task_weight, 
                                                                  target, 
                                                                  constr, 
                                                                  posWp, 
                                                                  oriWp,
                                                                  _bezier_curve_verbose_active);
  BSplineVel->stiffness(_magic_task_stiffness);
  BSplineVel->weight(_magic_task_weight);
  // std::cout << "BSplineVel->spline().get_bezier().constr_.end_vel.z(): " << BSplineVel->spline().get_bezier()->constr_.end_vel.z() << std::endl;
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


    if( BSplineVel->eval().norm() < _magic_epsilon)
    {
        output("Stop");
        return true;
    }
  
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

  ctl_.gui()->removeElement({}, _stop_hammering_button_name);

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

void Get_In_Position_Task::load_parameters()
{
  _nail_robot_name = "nail";
  _main_robot_name = "hrp5_p";
  _task_name = "Hammer::Get_In_Position_Task";

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  _config(gui_key)(stop_hammering_button_name_key, _stop_hammering_button_name);

  // ------------------------ Loading quality of life parameters ---------------------------

  std::string qol_key = "quality_of_life";
  std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
  _bezier_curve_verbose_active = _config(qol_key)(bezier_curve_verbose_active_key);


  // ------------------------ Loading frames ---------------------------

  std::string frames_key = "frames";
  std::string hammerhead_frame_key = "Hammer_Head";
  std::string nail_frame_key = "nail";
 _config(frames_key)(hammerhead_frame_key, _hammer_head_frame_name);
 _config(frames_key)(nail_frame_key, _nail_frame_name);

  // ------------------------ Loading magic values ---------------------------

  std::string magic_values_key = "magic_values";
  _magic_max_control_point_height = _config(magic_values_key)("max_control_point_height");
  _magic_bezier_curve_max_duration = _config(magic_values_key)("bezier_curve_max_duration");
  _magic_task_stiffness = _config(magic_values_key)("task_stiffness");
  _magic_task_weight = _config(magic_values_key)("task_weight");
  _magic_epsilon = _config(magic_values_key)("epsilon");
  _magic_oriWp_time = _config(magic_values_key)("oriWp_time");

  // ------------------------ Loading init and start velocities, accelerations and jerks ---------------------------

  std::string curve_constraints_key = "curve_constraints";
  std::string linear_velocity_key = "linear_velocity";
  std::string linear_acceleration_key = "linear_acceleration";
  std::string linear_jerk_key = "linear_jerk";
  std::string x_key = "x";
  std::string y_key = "y";
  std::string z_key = "z";

  std::string init_key = "init";
  std::string end_key = "end";

  constr.init_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(init_key);
  constr.init_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(init_key);
  constr.init_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(init_key);

  constr.init_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(init_key);
  constr.init_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(init_key);
  constr.init_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(init_key);

  constr.init_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(init_key);
  constr.init_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(init_key);
  constr.init_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(init_key);

  constr.end_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(end_key);
  constr.end_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(end_key);
  constr.end_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(end_key);

  constr.end_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(end_key);
  constr.end_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(end_key);
  constr.end_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(end_key);

  constr.end_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(end_key);
  constr.end_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(end_key);
  constr.end_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(end_key);
}

EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)