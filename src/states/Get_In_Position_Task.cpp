#include "Get_In_Position_Task.h"

#include "../Hammering_FSM_Controller.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <array>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <map>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PositionTask.h>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

// #include <pinocchio/parsers/urdf.hpp>


void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
    // mc_rtc::log::info("Get_In_Position_Task configure function called with config : \n{}", config.dump(true, true));
    
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  load_parameters();
  _dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(ctl.robots(), 
                                                                        ctl.robot().robotIndex(),
                                                                      ctl.solver().dt(),
                                                                      std::array<double, 3>{0.1, 0.01, 0.5},
                                                                      1.0,
                                                                    true);
  mc_rtc::log::info("solver timestep = {} s", ctl.solver().dt());
  _nrdof = ctl.robot().mb().nrDof();
  _old_mbc = ctl.robot().mbc();
  _old_effective_mass_mbc = compute_effective_mass_with_mbc(_old_mbc, ctl_, _normal_vector);
  _old_effective_mass_encoders = compute_effective_mass_with_encoders(ctl_.robot().encoderValues(), ctl_, _normal_vector);

  // for (size_t i = 0; i < std::size(ctl.robot().mbc().q); ++i)
  // {
  //   std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
  //   std::cout << "{";
  //   for(size_t j = 0; j < std::size(ctl.robot().mbc().q.at(i)); ++j)
  //   { 
  //     std::cout << ctl.robot().mbc().q.at(i).at(j) << ", ";
  //     // n+=1;
  //     // Q.push_back(qf.q.at(i).at(j));
  //   }
  //   std::cout << "}" << std::endl;
  // }
  // std::cout << "}" << std::endl;
  // std::cout << "robot pos world = \n" << ctl.robot().posW() << std::endl;
  // Add a stop button to the gui
  ctl.gui()->addElement({}, mc_rtc::gui::Button(_stop_hammering_button_name, [this]() { stop = true; }));
  
  ctl.getPostureTask(ctl.robot().name())->selectUnactiveJoints(ctl_.solver(), 
{
"WP",
"WR",
"WY"
});
  // Get the initial conditions
  // for (const auto &frame : ctl.robot().frames())
  // {
  //   std::cout << frame << "\n";
  // }
  // std::cout << "\n";
  // std::cout << ctl.robot().frame(_hammer_head_frame_name).body() << std::endl;
  // std::cout << ctl.robot().frame("LeftHand").position().rotation() << std::endl;

  // std::cout << "base frame rotation matrix = \n" << ctl.robot().frame("base_link").position().rotation() << "\n";
  printConfig(ctl.robot().mbc(), "q_init");
  // std::cout << "initial Hammer_head rotation : " <<_initial_hammerhead_position.rotation() << std::endl;

  _start_point = ctl.robot().frame(_hammer_head_frame_name).position().translation();
  _end_point = ctl.robots().robot(_nail_robot_name).frame(_nail_frame_name).position().translation();


  // std::cout << ctl.robots().robot(_nail_robot_name).frame(_nail_frame_name).position().rotation() << std::endl;
  // auto eigenvector = (totalRot - id).colPivHouseholderQr().solve(zero);

  // Found by calculations by hand
  _rotation_axis = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
  auto angle = M_PI;
  // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
  // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, _rotation_axis));

  _posWp ={_start_point,
          _end_point, 
          };
  
  
  // double bezier_duration = std::abs(_constr.end_vel.z()/_constr.end_acc.z());
  _oriWp = {
    std::make_pair(_magic_oriWp_time, q.matrix()) // at t = _magic_oriWp_time, arbitrary for now
  };

  // const sva::PTransformd & target = sva::PTransformd(Eigen::Quaterniond({0, -0.7, 0, 0.7}), nail_pos.translation());
  _target = sva::PTransformd(q, //maybe the quaternion expresses the orientation of your rotating frame you want to achieve at the end with respect to the world frame
                                                            //thus whatever the starting orientation of the rotating frame wrt the world frame, the robot frame will try to end up at the orientation specified by the quaternion
                                                            //how does it do that ?
                                                  // initial_nail_translation
                            _end_point
                            );
  
  // See if the task is tracked well. If it is tracked well but there is still a problem, then it might come from the trajectory.
  // If the trajectory is good but the tracking says otherwise, then there might be a problem with the tracking
  // Check if the hammer mass was correctly taken into account
  BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(_hammer_head_frame_name),
                                                                  // _magic_bezier_curve_max_duration, 
                                                                  _magic_bezier_curve_max_duration,
                                                                  _magic_task_stiffness, 
                                                                  _magic_task_weight, 
                                                                  _target, 
                                                                  _constr, 
                                                                  _posWp, 
                                                                  _oriWp,
                                                                  _bezier_curve_verbose_active);
  // // BSplineVel->stiffness(_magic_task_stiffness);
  // BSplineVel->weight(_magic_task_weight);
  // BSplineVel->selectActiveJoints(
  //       {
  //         "LWRR",
  //       "LWRP",
  //       "LWRY",
  //       "LSP",
  //       "LSR",
  //       "LSY",
  //       "LEP",  
  //     });
  // std::cout << "Degree of spline = " << BSplineVel->spline().get_bezier()->degree() << std::endl;
  // ctl.getPostureTask(ctl.robot().name())->weight(_posture_task_weight);
  // ctl.solver().removeTask(ctl.getPostureTask(ctl.robot().name()));
  // ctl.solver().addTask(BSplineVel);
  // _positionTask = std::make_shared<mc_tasks::PositionTask>(ctl.robot().frame(_hammer_head_frame_name), 2.0, 10);
  // _positionTask->position(_end_point);
  // ctl.solver().addTask(_positionTask);
  ctl.solver().addConstraintSet(_dynamicsConstraint);

  // mc_rbdyn::Robot cloned(ctl.robot());
  // std::cout << "torque = {" << std::endl;
  // for(const auto &joint : ctl.robot().mbc().jointTorque)
  // {
  //   std::cout << "{";
  //   for(const double &torque : joint)
  //   {
  //     std::cout << torque << ", ";
  //   }
  //   std::cout << "}" << std::endl;
  // }
  // std::cout << "}" << std::endl;

  _old_q_encoders = ctl_.robot().encoderValues();
  _old_reduced_effective_mass_mbc = compute_reduced_effective_mass_with_mbc(ctl_.robot().mbc(), ctl_, _normal_vector);
  _old_floating_base_effective_mass_mbc = _old_effective_mass_mbc - _old_reduced_effective_mass_mbc;
}




bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
  
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  _new_mbc = ctl_.robot().mbc();
  // mc_rtc::log::info("_nrdof = \n {}", _nrdof);
  Eigen::VectorXd gradient_of_m = compute_emass_gradient_central_difference_mbc(_new_mbc, 
                                                                              ctl, 
                                                                    _normal_vector);
  // mc_rtc::log::info("Gradient of m = {}", gradient_of_m);
  // Eigen::VectorXd gradient_of_m_encoders = compute_emass_gradient_central_difference_encoders(ctl_.robot().encoderValues(), 
  //                                                                                                       ctl, 
  //                                                                                             _normal_vector);

    double effective_mass_mbc = compute_effective_mass_with_mbc(_new_mbc, ctl_, _normal_vector);
  //   double effective_mass_encoders = compute_effective_mass_with_encoders(ctl_.robot().encoderValues(), ctl_, _normal_vector);
  //   double reduced_effective_mass_mbc = compute_reduced_effective_mass_with_mbc(_new_mbc, ctl_, _normal_vector);
    // compare_configs(gradient_of_m, ctl_);
  //   double floating_base_effective_mass_mbc = effective_mass_mbc - reduced_effective_mass_mbc;
  //   // mc_rtc::log::info("effective_mass_floating_base = {} kg", floating_base_effective_mass);

  //   // mc_rtc::log::info("dm_floating_base/dt = {} kg/s", (floating_base_effective_mass - _old_floating_base_effective_mass)/ctl_.solver().dt());
  //   // _old_floating_base_effective_mass = floating_base_effective_mass;
  //   mc_rtc::log::info("old_effective_mass_mbc = {} kg", _old_effective_mass_mbc);
    mc_rtc::log::info("Effective mass mbc of the robot = {} kg", effective_mass_mbc);
  //   mc_rtc::log::info("Reduced Effective mass mbc of the robot = {} kg", reduced_effective_mass_mbc);
  //   mc_rtc::log::info("Effective mass mbc of the floating base = {} kg", floating_base_effective_mass_mbc);
  //   mc_rtc::log::info("dm(floating base)/dt = {} kg/s", (floating_base_effective_mass_mbc - _old_floating_base_effective_mass_mbc)/ ctl_.solver().dt());
  //   mc_rtc::log::info("dm(reduced)/dt = {} kg/s", (reduced_effective_mass_mbc - _old_reduced_effective_mass_mbc)/ ctl_.solver().dt());
    // compute_effective_mass_naive(_new_mbc, ctl_);
    
    double dmdt_mbc = (effective_mass_mbc - _old_effective_mass_mbc) / ctl.solver().dt();
  //   double dmdt_encoders = (effective_mass_encoders - _old_effective_mass_encoders ) / ctl.solver().dt();
    
    mc_rtc::log::info("dm/dt mbc by finite difference = {} kg/s", dmdt_mbc);
  //   mc_rtc::log::info("Effective mass encoders of the robot = {} kg", effective_mass_encoders);
  //   mc_rtc::log::info("dm/dt encoders by finite difference = {} kg/s", dmdt_encoders);
    mc_rtc::log::info("dm/dt mbc by product with q_mbc_derivative = {} kg/s", emass_time_derivative_with_q_derivative(gradient_of_m, ctl_));
  //   mc_rtc::log::info("dm/dt encoders by product with q_dot_encoders = {} kg/s", emass_time_derivative_with_encoders(gradient_of_m, ctl_));
  //   mc_rtc::log::info("dm/dt mbc by product with mbc.alpha = {} kg/s", emass_time_derivative_with_mbc_alpha(gradient_of_m, ctl_));
    _old_mbc = _new_mbc;
    _old_effective_mass_mbc = effective_mass_mbc;
  //   _old_effective_mass_encoders = effective_mass_encoders;    
  //   _old_floating_base_effective_mass_mbc = floating_base_effective_mass_mbc;
  //   _old_reduced_effective_mass_mbc = reduced_effective_mass_mbc;
  //   // Reset FK
  //   ctl_.robot().forwardKinematics(_new_mbc);
    
  
    // mc_rtc::log::info("posture task ref accel = {}", ctl.getPostureTask(ctl.robot().name())->refAccel());
    ctl.getPostureTask(ctl.robot().name())->refAccel((_effective_mass_maximization_task_weight/(_posture_task_weight*_posture_task_weight)) * gradient_of_m);
    
    // mc_rtc::log::info("Posture objective = {");
    // for (size_t i = 0; i < std::size(ctl.getPostureTask(ctl.robot().name())->posture()); ++i)
    // {
    //   for(size_t j = 0; j < std::size(ctl.getPostureTask(ctl.robot().name())->posture().at(i)); ++j)
    //   {
    //     mc_rtc::log::info("[{}]", ctl.getPostureTask(ctl.robot().name())->posture().at(i).at(j));
    //   }
    // }
    // mc_rtc::log::info("}");
    // mc_rtc::log::info("Current Posture = {");
    // for (size_t i = 0; i < std::size(ctl.robot().mbc().q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(ctl.robot().mbc().q.at(i)); ++j)
    //   {
    //     mc_rtc::log::info("[{}]", ctl.robot().mbc().q.at(i).at(j));
    //   }
    // }
    // mc_rtc::log::info("}");
    // mc_rtc::log::info("Posture refVel = {}", ctl.getPostureTask(ctl_.robot().name())->);
    // mc_rtc::log::info("Posture refVel = {}", ctl.getPostureTask(ctl_.robot().name())->refVel());
    // mc_rtc::log::info("Posture refAcc = {}", ctl.getPostureTask(ctl_.robot().name())->refAccel());
    // mc_rtc::log::info("Posture stiffness = {}", ctl.getPostureTask(ctl_.robot().name())->stiffness());
    // mc_rtc::log::info("Posture damping = {}", ctl.getPostureTask(ctl_.robot().name())->damping());
// ctl.getPostureTask(ctl_.robot().name())->
    // mc_rtc::log::info("Gradient of m = \n {}", gradient_of_m);
    // mc_rtc::log::info("Gradient of m_encoders = \n {}", gradient_of_m_encoders);
    
    auto tasks = ctl.solver().tasks();
    for(const auto & task : tasks)
    {
      mc_rtc::log::info("Task {}", task->name());
    }
    // BSplineVel->weight(10/BSplineVel->eval().norm());
    // return true;
    // mc_rtc::log::info("eval norm = {} ", BSplineVel->eval().norm());
    // ctl.getPostureTask(ctl.robot().name())->refAccel((_effective_mass_maximization_task_weight/(_posture_task_weight*_posture_task_weight)) * gradient_of_m);
    // mc_rtc::log::info("---------------------------");

    if (_create_file)
    {
      mc_rtc::log::info("Effective mass of the robot at the nail = {} kg", compute_effective_mass_with_mbc(ctl.robot().mbc(), ctl_, _normal_vector));
      _create_file = false;
    }
    
    if( BSplineVel->eval().norm() < _magic_epsilon)
    {
      _effective_mass = compute_effective_mass_naive(ctl.robot().mbc(), ctl_);
      // writeVectorToCSV(_masses, "Masses_vector.csv");
      // mc_rtc::log::info("Effective mass = {} kg", _effective_mass);
      output("STOP");
      return true;
    }
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto       // backward_mbc.q.at(i).at(0) -= 1000;
& ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

  ctl_.gui()->removeElement({}, _stop_hammering_button_name);

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

void Get_In_Position_Task::compare_configs(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_)
{
  std::vector<double> q_encoders = ctl_.robot().encoderValues();
  auto q_mbc = ctl_.robot().mbc().q;
  std::vector<double> q_dot_encoders = ctl_.robot().encoderVelocities();
  auto q_dot_mbc = ctl_.robot().mbc().alpha;
  auto ref_joint_order = ctl_.robot().refJointOrder();
  std::map<std::string, double> q_mbc_map;
  std::map<std::string, double> q_dot_mbc_map;
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, double> q_dot_encoders_map;
  std::map<std::string, double> q_dot_encoders_derivative_map;
  std::map<std::string, double> q_dot_mbc_derivative_map;
  std::map<std::string, double> q_mbc_difference_map;
  std::map<std::string, double> q_encoders_difference_map;
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);

  for(size_t i = 1; i < std::size(q_mbc); ++i)
  {
    if (q_mbc.at(i).empty()) {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = 0;
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = 0;

    }
    else {
      q_mbc_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0);
      q_dot_mbc_map[ctl_.robot().mb().joint(i).name()] = q_dot_mbc.at(i).at(0);
      q_mbc_difference_map[ctl_.robot().mb().joint(i).name()] = q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0);
      q_dot_mbc_derivative_map[ctl_.robot().mb().joint(i).name()] = (q_mbc.at(i).at(0) - _old_mbc.q.at(i).at(0))/ctl_.solver().dt();
    }
  }

  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
    q_dot_encoders_map[ref_joint_order.at(i)] = q_dot_encoders.at(i);
    q_encoders_difference_map[ref_joint_order.at(i)] = q_encoders.at(i) - _old_q_encoders.at(i);
    q_dot_encoders_derivative_map[ref_joint_order.at(i)] = (q_encoders.at(i) - _old_q_encoders.at(i))/ctl_.solver().dt();
  }

  mc_rtc::log::info("Q"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc, q_encoders =  [{}, {}]", 
      jointName, 
      q_mbc_map.at(jointName),
      q_encoders_map.at(jointName)
      );
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_difference"); 

  for(const std::string &jointName : ref_joint_order)
  {
    mc_rtc::log::info("({}) q_mbc_difference, q_encoders_difference =  [{}, {}]", 
      jointName, 
      q_mbc_difference_map.at(jointName),
      q_encoders_difference_map.at(jointName)); 
  }
  mc_rtc::log::info("---------------------"); 
  mc_rtc::log::info("Q_DOT"); 

  uint8_t i = 0;
  for(const std::string &jointName : ref_joint_order)
  {
  mc_rtc::log::info("({}) q_dot_mbc, q_dot_mbc_der, q_dot_encoders, q_dot_encoders_der, grad_m =  [{}, {}, {}, {}, {}]", 
    jointName, 
    q_dot_mbc_map.at(jointName),
    q_dot_mbc_derivative_map.at(jointName),
    q_dot_encoders_map.at(jointName),
    q_dot_encoders_derivative_map.at(jointName),
    reorg_gradient(i, 0)
  );
  ++i;
  }
  _old_q_encoders = q_encoders;
}
const double Get_In_Position_Task::emass_time_derivative_with_encoders(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  Eigen::VectorXd reorg_gradient = reorganized_gradient(gradient, ctl_);
  // std::vector<std::vector<double>> qdot_rbdyn = _new_mbc.alpha;
  auto q_dot_encoders = ctl_.robot().encoderVelocities();
  // for(size_t i = 0; i < std::size(q_encoders); ++i)
  // {

  //   mc_rtc::log::info("({}) q_dot_encoders[{}], reorg_gradient[{}] = [{}, {}]",
  //     joint_order[i], 
  //     i,
  //     i,
  //     q_dot_encoders.at(i),
  //   reorg_gradient(i, 0));
  //   if (i < 26 || i > 34 && i < 44)
  //   {
  //     // Ignore the gripper joint
  //     q_dot_norm += q_dot_encoders.at(i) * q_dot_encoders.at(i);
  //   }
  // }
  // q_dot_norm = std::sqrt(q_dot_norm);
  //  mc_rtc::log::info("Norm of q_dot = {}", q_dot_norm);

  double dmdt = 0;
  for(size_t i = 0; i < std::size(q_dot_encoders); ++i)
  {
    dmdt += reorg_gradient(i, 0)*q_dot_encoders.at(i);
  }
  return dmdt;
}

const Eigen::VectorXd Get_In_Position_Task::reorganized_gradient(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  // mc_rtc::log::info("grad size = {}", std::size(gradient));
  Eigen::VectorXd res(std::size(ctl_.robot().encoderVelocities()), 1);
  res.setOnes();
  for(size_t i = 0; i < std::size(ctl_.robot().encoderVelocities()); ++i)
  {
    if(i >= 0 && i < 6)
    {
      res(i, 0) = gradient(i+12, 0);
    }
    else if (i >= 6 && i < 12) 
    {  
      res(i, 0) = gradient(i, 0);
    }
    else if (i >= 12 && i < 17) 
    {
      res(i, 0) = gradient(i+6, 0);
    }
    else if (i >= 17 && i < 26) 
    {
      res(i, 0) = gradient(i+15, 0);
    }
    else if (i>= 26 && i < 35) 
    {
      res(i, 0) = 0;
    }
    else if (i >= 35 && i < 44) 
    {
      res(i, 0) = gradient(i-12, 0);
    }
    else if(i >= 44 && i < std::size(ctl_.robot().encoderVelocities()))
    {
      res(i, 0) = 0;
    }
  }
  return res;
}

const double Get_In_Position_Task::emass_time_derivative_with_q_derivative(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  double dmdt = 0;
  unsigned int j = 0; // Skip the 6 first rows representing the floating base
  // unsigned int l = 6; // Skip the 6 first rows representing the floating base

  std::vector<std::vector<double>> q = _new_mbc.q;
  std::vector<std::vector<double>> old_q = _old_mbc.q;
  
  // mc_rtc::log::info("qdot alpha, qdot num deriv, grad = {");
  // for(size_t i = 0; i < std::size(qdot); ++i)
  // {
  //   if(std::size(qdot.at(i)) != 1)
  //   {
  //     mc_rtc::log::info("({}) = []", ctl_.robot().mb().joint(i).name());
  //     continue;
  //   }
  //   mc_rtc::log::info("({}) alpha, qdot_num_deriv, grad_m[{}] = [{}, {}, {}]", 
  //     ctl_.robot().mb().joint(i).name(), 
  //     l, 
  //     qdot.at(i).at(0), 
  //     (q.at(i).at(0) - old_q.at(i).at(0))/ctl_.solver().dt(),
  //     gradient(l, 0));
  //   l+=1;
  // }
  // mc_rtc::log::info("}");  
  // std::cout << "l = " << l << std::endl;

  // for (size_t i = 0; i < 6; ++i)
  // {  
  //   dmdt += gradient(i, 1)*qdot.at(0).at(i);
  // }
  for (size_t i = 0; i < std::size(q); ++i)
  {
    if (std::size(q.at(i)) != 1)
    {
      //Ignore floating base and empty dofs
      continue;
    }    
    // dmdt += gradient(j, 0)*qdot.at(i).at(0);

    dmdt += gradient(j,0)*((q.at(i).at(0) - old_q.at(i).at(0))/ctl_.solver().dt());
    j+=1;
  }
  // std::cout << "j = " << j << std::endl;

  return dmdt;
}
const double Get_In_Position_Task::emass_time_derivative_with_mbc_alpha(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  double dmdt = 0;
  unsigned int j = 6; // Skip the 6 first rows representing the floating base

  std::vector<std::vector<double>> qdot = _new_mbc.alpha;
  
  for (size_t i = 0; i < std::size(qdot); ++i)
  {
    if (std::size(qdot.at(i)) != 1)
    {
      //Ignore floating base and empty dofs
      continue;
    }    
    dmdt += gradient(j, 0)*qdot.at(i).at(0);

    j+=1;
  }

  return dmdt;
}
const double Get_In_Position_Task::compute_reduced_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                                            mc_control::fsm::Controller & ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    
  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, mbc);
  Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  // .bottomRows(35).rightCols(35);
  // full_world_frame_jacobian = full_world_frame_jacobian.rightCols(35);
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::MatrixXd M_inverse_reduced = M_inverse.block(6, 6, _nrdof-6, _nrdof-6);
  Eigen::MatrixXd linear_jacobian_reduced = linear_jacobian.block(0, 6, 3, _nrdof-6);
  Eigen::MatrixXd LAMBDA_reduced = linear_jacobian_reduced*M_inverse_reduced*linear_jacobian_reduced.transpose();
  return 1/(normal_vector.transpose()*LAMBDA_reduced*normal_vector);

}
const double Get_In_Position_Task::compute_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                                            mc_control::fsm::Controller & ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    
  // If you dont put this line the gradient is 0 everywhere because M and J are not updating
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  const Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();

  const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  // mc_rtc::log::info("Dim of M = {} x {}", M.cols(), M.rows());
  const Eigen::MatrixXd M_inverse = M.inverse();
  const Eigen::MatrixXd LAMBDA = linear_jacobian*M_inverse*linear_jacobian.transpose();
  // writeEigenMatrixToCSV(M, "ReducedMassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "Reducedfull_world_frame_jacobian.csv");
  // return 1/(normal_vector.transpose()*LAMBDA*normal_vector) - FLOATING_BASE_EFFECTIVE_MASS;
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);

}
const double Get_In_Position_Task::compute_floating_base_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                                            mc_control::fsm::Controller & ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{


  // NOT FINISHED !!!!
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  const Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();

  const Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  const Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::MatrixXd M_inverse_reduced = M_inverse.leftCols(6);
  Eigen::MatrixXd linear_jacobian_floating_base_part = linear_jacobian.leftCols(6);
  const Eigen::MatrixXd FLOATING_BASE_LAMBDA = linear_jacobian_floating_base_part*M_inverse_reduced*linear_jacobian_floating_base_part.transpose();
  
  const double FLOATING_BASE_EFFECTIVE_MASS = 1/(normal_vector.transpose()*FLOATING_BASE_LAMBDA*normal_vector);
  return FLOATING_BASE_EFFECTIVE_MASS;
}


const double Get_In_Position_Task::compute_effective_mass_with_encoders(const std::vector<double> &encoderValues,
                                                            mc_control::fsm::Controller & ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    
  rbd::MultiBodyConfig mbc = ctl_.robot().mbc();

  std::vector<double> q_encoders = encoderValues;
  std::map<std::string, std::vector<double>> q_mbc_map;
  std::vector<std::string> ref_joint_order = ctl_.robot().refJointOrder();
  std::map<std::string, double> q_encoders_map;
  std::map<std::string, std::vector<double>> res;
  
  // mc_rtc::log::info("mbc.q before modification :");
  // for(size_t i = 0; i < std::size(mbc.q); ++i)
  // {
    //   std::cout << "{";
    //   for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
    //   {
      //     std::cout << mbc.q.at(i).at(j) << ",";
      //   }
      //   std::cout << "}" << "\n";
      // }
      
  //Convert the encoderValues vector into some mbc.q vector
  for(size_t i = 0; i < std::size(q_encoders); ++i)
  {
    q_encoders_map[ref_joint_order.at(i)] = q_encoders.at(i);
  }

  for(size_t i = 0; i < std::size(mbc.q); ++i)
  {
    q_mbc_map[ctl_.robot().mb().joint(i).name()] = mbc.q.at(i);
  }

  for(size_t i = 0; i < std::size(mbc.q); ++i)
  {
    std::string jointName = ctl_.robot().mb().joint(i).name();
    try{
      if (std::size(q_mbc_map.at(jointName)) != 1)
      {
        res[jointName] = q_mbc_map.at(jointName);
      }
      else
      {
        res[jointName] = {q_encoders_map.at(jointName)};
      }
    }
    catch (std::out_of_range){
      res[jointName] = q_mbc_map.at(jointName);
    }
    mbc.q[i] = res.at(jointName);
  }
  // std::cout << "------------------" << std::endl;

  // // mc_rtc::log::info("mbc.q after modification :");
  // for(size_t i = 0; i < std::size(mbc.q); ++i)
  // {
  //   std::cout << "(" << ctl_.robot().mb().joint(i).name() << ") : " << "{";
  //   for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
  //   {
  //     std::cout << mbc.q.at(i).at(j) << ",";
  //   }
  //   std::cout << "}" << "\n";
  // }
  ctl_.robot().forwardKinematics(mbc);


  rbd::MultiBody robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot_mb, mbc);

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, mbc);
  Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  // .bottomRows(35).rightCols(35);
  // full_world_frame_jacobian = full_world_frame_jacobian.rightCols(35);
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::MatrixXd LAMBDA = linear_jacobian*M_inverse*linear_jacobian.transpose();
  // writeEigenMatrixToCSV(M, "ReducedMassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "Reducedfull_world_frame_jacobian.csv");
  return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_backward_difference_mbc(rbd::MultiBodyConfig mbc, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{
  const double epsilon = 0.0000001; 
  double dq = 0;
  Eigen::VectorXd grad(_nrdof, 1);
  grad.setOnes();
  double m_q = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);
  double backward_effective_mass = 0;
  for(size_t i = 0; i < 6; ++i)
  {
    //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
    grad(i,0) = 0;
  }
  
  unsigned int j = 6; // Skipping the first 6 rows representing the floating base
  rbd::MultiBodyConfig backward_mbc;
  for(size_t i = 0; i <  std::size(mbc.q); ++i)
  { 
    if(mbc.q.at(i).size() != 1)
    {
      // Ignore floating base and fixed joints
      continue;
    }
      dq = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
      backward_mbc = mbc;
      backward_mbc.q.at(i).at(0) -= dq;

      //Finite differences
      backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, ctl_, normal_vector);
      grad(j,0) = (m_q - backward_effective_mass)/dq;
      j+=1;
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");
      // mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);
      // backward_mbc.q.at(i).at(0) = mbc.q.at(i).at(0);
    }
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_mbc(rbd::MultiBodyConfig mbc, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

    double dq = 0.0000001; 
    Eigen::VectorXd grad(_nrdof, 1);
    grad.setOnes();

    // for(size_t i = 0; i < 6; ++i)
    // {
    //   //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
    //   grad(i,0) = 0;
    // }

    rbd::MultiBodyConfig forward_mbc;
    rbd::MultiBodyConfig backward_mbc;

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;
    unsigned int j = 0; // Skipping the first 6 rows representing the floating base
    for(size_t i = 0; i <  std::size(mbc.q); ++i)
    { 
      if(mbc.q.at(i).size() != 1)
      {
        // Ignore floating base and fixed joints
        continue;
      }
      dq = mbc.q.at(i).at(0) - _old_mbc.q.at(i).at(0);
      forward_mbc = mbc;
      backward_mbc = mbc;
      forward_mbc.q.at(i).at(0) += dq;
      backward_mbc.q.at(i).at(0) -= dq;
      // forward_mbc.q.at(i).at(0) += 1000;
      // backward_mbc.q.at(i).at(0) -= 1000;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_mbc(forward_mbc, 
                                                                ctl_, 
                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_mbc(backward_mbc, 
                                                                  ctl_, 
                                                                  normal_vector);


      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*dq);
      j+=1;
    }
  
  return grad;
}

const Eigen::VectorXd Get_In_Position_Task::compute_emass_gradient_central_difference_encoders(
                                                            const std::vector<double> &encoderValues, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

    // NOT FINISHED !!!!

    const double epsilon = 0.0000001; 
    Eigen::VectorXd grad(_nrdof, 1);
    grad.setOnes();

    std::vector<std::string> unusable_dofs;
    for (size_t i = 0; i < std::size(ctl_.robot().mbc().q); ++i)
    {
      if (std::size(ctl_.robot().mbc().q.at(i)) != 1)
      {
        unusable_dofs.push_back(ctl_.robot().mb().joint(i).name());
      }
    }
    // std::map<std::string, double> encoder_values_map;
    // for (size_t i = 0; i < std::size(encoderValues); ++i)
    // {
    //   encoder_values_map[ctl_.robot().refJointOrder().at(i)] = encoderValues.at(i);
    // }
    // std::vector<double> reorg_encoder_values;
    // for (size_t i = 0; i < std::size(usable_dofs); ++i)
    // {
    //   reorg_encoder_values.at(i) = encoder_values_map.at(usable_dofs.at(i));
    // }    

    double forward_effective_mass = 0;
    double backward_effective_mass = 0;

    for(size_t i = 0; i < 6; ++i)
    {
      //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
      grad(i,0) = 0;
    }
    unsigned int j = 6; // Skipping the first 6 rows representing the floating base
    for(size_t i = 0; i <  std::size(encoderValues); ++i)
    { 
      // What dof i am touching : if it is an unactuated dof i should skip
      if (std::find(unusable_dofs.begin(), 
                      unusable_dofs.end(), 
                      ctl_.robot().refJointOrder().at(i)) != unusable_dofs.end())
                      //refJointOrder should be the same as the mbc order ?
      {
        continue;
      }
      std::vector<double> forward_encoder_values = encoderValues;
      std::vector<double> backward_encoder_values = encoderValues;
      
      forward_encoder_values.at(i) += epsilon;
      backward_encoder_values.at(i) -= epsilon;

      //Finite differences - Central differences
      forward_effective_mass = compute_effective_mass_with_encoders(forward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      backward_effective_mass = compute_effective_mass_with_encoders(backward_encoder_values, 
                                                                                ctl_, 
                                                                                  normal_vector);
      grad(j,0) = (forward_effective_mass - backward_effective_mass)/(2*epsilon);
      j+=1;
      
    }
  
  return grad;
}
const double Get_In_Position_Task::compute_effective_mass_naive(rbd::MultiBodyConfig mbc, mc_control::fsm::Controller & ctl_) const
{
  // ctl_.robot().forwardKinematics(mbc);
  // auto robot_mb = ctl_.robot().mb();
  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, q_test);
  // double delta_psi = 0.01; // rad

  // std::cout << "nrDOF = " << _nrdof << std::endl;
  // std::cout << "size = " << std::size(qf.q) << std::endl;
  // int n = 0;
  std::cout << "qf = {"  << std::endl;

  // Store only the actuated dofs in Q
  // std::vector<double> Q = {};
  for (size_t i = 0; i < std::size(mbc.q); ++i)
  {
    std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
    std::cout << "{";
    for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
    {
      
      std::cout << mbc.q.at(i).at(j) << ", ";
  //     n+=1;
  //     Q.push_back(qf.q.at(i).at(j));
    }
    std::cout << "}" << std::endl;
  }
  std::cout << "}" << std::endl;
  // std::cout << "n = " << n << std::endl;

  // std::cout << "Q = {"  << std::endl;
  // for (size_t i = 0; i < std::size(Q); ++i){
  //   std::cout << Q.at(i) << std::endl;
  // }
  // std::cout << "}"  << std::endl;
  // Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  //   // std::cout << "M  = \n" << M << "\n";
  // Eigen::MatrixXd M_inverse = M.inverse();
  // // std::cout << "Creating my Jacobian" << std::endl;
  // rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  // auto world_frame_jacobian = jac.jacobian(robot_mb,mbc);
  // // std::cout << "Created world frame jacobian" << std::endl;

  // Eigen::MatrixXd full_world_frame_jacobian(6, ctl_.robot().mb().nrDof());
  // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
  // // std::cout << "Projected my world frame Jacobian" << std::endl;
  
  // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created linear jacobian" << std::endl;

  // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);
  // std::cout << "Created angular jacobian" << std::endl;

  // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
  // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
  // std::cout << "Created pseudo inverse" << std::endl;
  
  // auto pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);
  // std::cout << "Created pseudo inverse angular jac" << std::endl;

  // Eigen::MatrixXd delta_x_omega(3, 1);
  // std::cout << "Created delta x omega" << std::endl;

  // delta_x_omega << 0, 0, delta_psi;

  // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
  // std::cout << "Created delta q" << std::endl;e

  // std::cout << "JointConfig = { " << std::endl;
  // for(size_t i = 0; i < std::size(qf.jointConfig); ++i)
  // {
  //   std::cout << qf.jointConfig.at(i) << std::endl;
  // }
  // std::cout << "}" << std::endl;

  // std::cout << "delta q = " << delta_q << std::endl;

  // auto incremented_robot_config = qf.q;
  // incremented_robot_config.at(37);
  
  // std::cout << "Joint " << ctl_.robot().mb().jointIndexByName("LWRY") << std::endl;
  // std::cout << "Joint value at 37 : " << Q.at(37 + 1) << std::endl;

  
  // std::cout << M_inverse << std::endl;
  // std::cout << "Created M inverse" << std::endl;
  // std::cout << "Size M^-1 = " << M_inverse.rows() << " x " << M_inverse.cols() << std::endl;
  // writeEigenMatrixToCSV(M, "MassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "full_world_frame_jacobian.csv");
  // std::cout << M << std::endl;
  
  
  
  double inner_sum = 0;
  double J_nu_3_j = 0;
  double J_nu_3_k = 0;
  double M_inverse_j_k = 0;
  
  double outer_sum = 0;
  double effective_mass = 0;
  double delta_psi_tot = 0;
  // auto q_start = qf;
  // std::cout << "Starting double sum" << std::endl;
  
  // while(delta_psi_tot < 2*M_PI)
  // {
    // std::cout << "delta_psi_tot = " << delta_psi_tot << std::endl;
    //Create everything

    // std::cout << M_inverse.row(0) << std::endl;
    // world_frame_jacobian = jac.jacobian(robot_mb,q_test);

    // Eigen::MatrixXd full_world_frame_jacobian(6, robot_mb.nrDof());
    // jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
    
    // Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

    // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);

    // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
    // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
    
    // pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);

    // Eigen::MatrixXd delta_x_omega(3, 1);

    // Eigen::MatrixXd pseudo_inverted_full_jac = full_world_frame_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(full_world_frame_jacobian.rows(), full_world_frame_jacobian.rows()));
    // delta_x_omega << 0, 0, delta_psi;
    // Eigen::MatrixXd delta_x(6, 1);
    // delta_x << 0, 0, 0, 0, 0, delta_psi;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_full_jac* delta_x;


    // Compute Outer sum
    // outer_sum = 0;
    // for(int k=0; k <= _nrdof - 1 ; ++k)
    // {
    //   // Inner sum
    //   inner_sum = 0;
    //   for(int j=0; j <= _nrdof - 1; ++j)
    //   { 
    //     J_nu_3_j = linear_jacobian(3-1,j);
    //     // std::cout << "Computed linear jac (3-1, "  << j  << ") = " << linear_jacobian(3-1, j) << std::endl;
    //     M_inverse_j_k = M_inverse(j, k);
    //     // std::cout << "M_inverse (" << j << "," << k << ") = " << M_inverse_j_k << std::endl;
    //     inner_sum += J_nu_3_j * M_inverse_j_k;
    //   }
    //   J_nu_3_k = linear_jacobian(3-1, k);
    //   outer_sum += inner_sum*J_nu_3_k;
    // }

    // std::cout << "Testing effective mass : "<< 1/outer_sum << std::endl;
    // if(1/outer_sum > effective_mass)
    // {
    //   effective_mass = 1/outer_sum;
    // }

    // std::cout << "deltaq =  " << delta_q << std::endl;

    // printConfig(q_test, "q_test before = {");        
    // increment q_test by delta q
    // int L = 0;
    // int m = 0;
    // for (size_t i = 0; i < std::size(q_test.q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(q_test.q.at(i)); ++j)
    //   {
    //     q_test.q.at(i).at(j) += delta_q(j+L, 0);
    //     m += 1;
    //   }
    //   L += m;
    //   m = 0;
    // }

    // printConfig(q_test, "q_test after = {");        
  //   delta_psi_tot += delta_psi;
  // }
  return 1/1;
}

void Get_In_Position_Task::printConfig(rbd::MultiBodyConfig mbc, std::string string) const
{
    std::cout << string << std::endl;
    for (size_t i = 0; i < std::size(mbc.q); ++i)
    {
      for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
      {
        std::cout << mbc.q.at(i).at(j) <<"," << std::endl;
      }
    }
    std::cout << "}" << std::endl;
}

void Get_In_Position_Task::writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const {
    // std::cout << "entering write eigen function" << std::endl;
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    // std::cout << "For loop writeEigen begin" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) file << ",";  // Separate columns by commas
        }
        file << "\n";  // Newline for each row
    }
    file.close();
    // std::cout << "Matrix written to " << filename << std::endl;
}
void Get_In_Position_Task::writeVectorToCSV(const std::vector<double> & vec, const std::string & filename) const 
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    for(size_t i = 0; i < vec.size(); ++i)
    {
        file << vec[i];
        if(i != vec.size() - 1)
            file << ",";  // comma between values
    }

    file << "\n";  // new line at the end
    file.close();
}

void Get_In_Position_Task::load_parameters()
{
  _nail_robot_name = "nail";
  _main_robot_name = "hrp5_p";
  // ------------------------ Loading timestep ---------------------------
  std::string timestep_key = "timestep";
  _timestep = _config(timestep_key);

  // ------------------------ Loading gui parameters ---------------------------

  std::string gui_key = "gui";
  std::string stop_hammering_button_name_key = "stop_hammering_button_name";
  _config(gui_key)(stop_hammering_button_name_key, _stop_hammering_button_name);

  // ------------------------ Loading quality of life parameters ---------------------------

  std::string qol_key = "quality_of_life";
  std::string jacobian_verbose_active_key = "jacobian_verbose_active";
  std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
  _bezier_curve_verbose_active = _config(qol_key)(bezier_curve_verbose_active_key);
  _jacobian_verbose_active = _config(qol_key)(jacobian_verbose_active_key);



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
  _posture_task_weight = _config(magic_values_key)("posture_task_weight");
  _effective_mass_maximization_task_weight = _config(magic_values_key)("effective_mass_maximization_task_weight");

  _magic_oriWp_time = _config(magic_values_key)("oriWp_time");
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

  _constr.init_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(init_key);
  _constr.init_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(init_key);
  _constr.init_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(init_key);

  _constr.init_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(init_key);
  _constr.init_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(init_key);
  _constr.init_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(init_key);

  _constr.init_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(init_key);
  _constr.init_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(init_key);
  _constr.init_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(init_key);

  _constr.end_vel.x() = _config(curve_constraints_key)(linear_velocity_key)(x_key)(end_key);
  _constr.end_vel.y() = _config(curve_constraints_key)(linear_velocity_key)(y_key)(end_key);
  _constr.end_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(end_key);

  _constr.end_acc.x() = _config(curve_constraints_key)(linear_acceleration_key)(x_key)(end_key);
  _constr.end_acc.y() = _config(curve_constraints_key)(linear_acceleration_key)(y_key)(end_key);
  _constr.end_acc.z() = _config(curve_constraints_key)(linear_acceleration_key)(z_key)(end_key);

  _constr.end_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(end_key);
  _constr.end_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(end_key);
  _constr.end_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(end_key);
}

EXPORT_SINGLE_STATE("Get_In_Position_Task", Get_In_Position_Task)