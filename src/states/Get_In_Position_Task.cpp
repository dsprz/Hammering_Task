#include "Get_In_Position_Task.h"

#include "../Hammering_FSM_Controller.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>
#include <array>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PositionTask.h>
#include <memory>
#include <ostream>
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
  _old_effective_mass = compute_effective_mass_2(_old_mbc, ctl_, _normal_vector);
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
  // BSplineVel->stiffness(_magic_task_stiffness);
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
  
}




bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto new_mbc = ctl_.robot().mbc();

    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    auto gradient_of_m = compute_gradient(new_mbc, ctl, _normal_vector);
    auto effective_mass = compute_effective_mass_2(new_mbc, ctl_, _normal_vector);
    mc_rtc::log::info("old_effective_mass = {} kg", _old_effective_mass);
    mc_rtc::log::info("Effective mass of the robot = {} kg", effective_mass);
    auto dmdt = (effective_mass - _old_effective_mass) / ctl.solver().dt();
    mc_rtc::log::info("Finite difference dm/dt = {} kg/s",dmdt);
    _old_effective_mass = effective_mass;
    mc_rtc::log::info("Product dm/dt = {} kg/s", mass_time_derivative(gradient_of_m, ctl_));
  
    // mc_rtc::log::info("posture task ref accel = {}", ctl.getPostureTask(ctl.robot().name())->refAccel());
    _masses.push_back(effective_mass);
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

    // mc_rtc::log::info("Weighted Gradient of m = \n {}", (_effective_mass_maximization_task_weight/_posture_task_weight) *gradient_of_m);
    auto tasks = ctl.solver().tasks();
    BSplineVel->weight(10/BSplineVel->eval().norm());
    // return true;
    for(const auto & task : tasks)
    {
      mc_rtc::log::info("Task {}", task->name());
    }
    // mc_rtc::log::info("eval norm = {} ", BSplineVel->eval().norm());
    // ctl.getPostureTask(ctl.robot().name())->refAccel((_effective_mass_maximization_task_weight/(_posture_task_weight*_posture_task_weight)) * gradient_of_m);

    if (_create_file)
    {
      mc_rtc::log::info("Effective mass of the robot at the nail = {} kg", compute_effective_mass_2(ctl.robot().mbc(), ctl_, _normal_vector));
      _create_file = false;
    }
    
    if( BSplineVel->eval().norm() < _magic_epsilon)
    {
      _effective_mass = compute_effective_mass(ctl.robot().mbc(), ctl_);
      writeVectorToCSV(_masses, "Masses_vector.csv");
      // std::cout << "Effective mass = " << _effective_mass << std::endl;
      output("STOP");
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

const double Get_In_Position_Task::mass_time_derivative(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const
{
  double dmdt = 0;
  int j = 0;
  auto qdot = ctl_.robot().mbc().alpha;
  for (size_t i = 0; i < std::size(qdot); ++i)
  {
    printConfig(ctl_.robot().mbc(), "q = {");
    if (std::size(qdot.at(i)) != 1)
    {
      continue;
    }

    // mc_rtc::log::info("grad [{}] = {}", j, gradient(j, 1));
    
    dmdt += gradient(j, 1)*qdot.at(i).at(0);
    j+=1;
  }
  return dmdt;
}

const double Get_In_Position_Task::compute_effective_mass_2(rbd::MultiBodyConfig mbc, 
                                                            mc_control::fsm::Controller & ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  // auto & robot = ctl_.robots()[0];

  // ctl_.robot().forwardKinematics(mbc);
  // robot.forwardKinematics();
    
  // printConfig(mbc, "q used = { \n");
  // If you dont put this line the gradient is 0 everywhere
  ctl_.robot().forwardKinematics(mbc);


  auto robot_mb = ctl_.robot().mb();
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  auto world_frame_jacobian = jac.jacobian(robot_mb, mbc);
  // std::cout << "Created world frame jacobian" << std::endl;

  Eigen::MatrixXd full_world_frame_jacobian(6, _nrdof);
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);

  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, mbc);
  Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
  // .bottomRows(35).rightCols(35);
    // std::cout << "M  = \n" << M << "\n";
  // full_world_frame_jacobian = full_world_frame_jacobian.rightCols(35);
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  Eigen::MatrixXd M_inverse = M.inverse();
  Eigen::MatrixXd LAMDA = linear_jacobian*M_inverse*linear_jacobian.transpose();
  // writeEigenMatrixToCSV(M, "ReducedMassMatrixMcRTC.csv");
  // writeEigenMatrixToCSV(full_world_frame_jacobian, "Reducedfull_world_frame_jacobian.csv");
  return 1/(normal_vector.transpose()*LAMDA*normal_vector);
}

const Eigen::VectorXd Get_In_Position_Task::compute_gradient(rbd::MultiBodyConfig mbc, 
                                                             mc_control::fsm::Controller &ctl_, 
                                                            const Eigen::Vector3d &normal_vector) const{

    // Store only the actuated dofs in Q
    // std::vector<double> Q = {};
    // for (size_t i = 0; i < std::size(mbc.q); ++i)
    // {
    //   for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
    //   {
        
    //     std::cout << "Pushing back : " << mbc.q.at(i).at(j) << std::endl;
        
    //     Q.push_back(mbc.q.at(i).at(j));
    //   }backward_effective_mass
    // }
    // std::cout << "nrDof = " << _nrdof << std::endl;
    const double epsilon = 0.00001; //0.06 deg
    Eigen::VectorXd grad(_nrdof, 1);
    grad.setOnes();
    for(size_t i = 0; i < 6; ++i)
    {
      //We suppose that the floating base does not move, hence the first 6 DOFs gradient wrt to q are 0
      grad(i,0) = 0;
    }

    int j = 0;
    for(size_t i = 0; i <  std::size(mbc.q); ++i)
    { 
      if(mbc.q.at(i).size() != 1)
      {
        // Ignore floating base and fixed joints
        // mc_rtc::log::info("mbc.q = {}", mbc.q.at(i).at(0));
        continue;
      }

      auto forward_mbc = mbc;
      auto backward_mbc = mbc;
      // std::cout << "initial_mbc_q[i] =  = " << mbc.q.at(i).at(0) << std::endl;
      // std::cout << "jointConfig = " << mbc.jointConfig << std::endl;


      forward_mbc.q.at(i).at(0) += epsilon;
      backward_mbc.q.at(i).at(0) -= epsilon;
      // forward_mbc.q.at(i).at(0) += 1000;
      // backward_mbc.q.at(i).at(0) -= 1000;

      //Finite differences
      double forward_effective_mass = compute_effective_mass_2(forward_mbc, ctl_, normal_vector);
      double backward_effective_mass = compute_effective_mass_2(backward_mbc, ctl_, normal_vector);
      
      // mc_rtc::log::info("forward_mbc_q[i] = {}", forward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("backward_mbc_q[i] = {}", backward_mbc.q.at(i).at(0));
      // mc_rtc::log::info("---------------------");

      double grad_res = (forward_effective_mass - backward_effective_mass)/(2*epsilon);
      mc_rtc::log::info("grad[{}] = {}", 6+j, grad_res);

      grad(6+j,1) = grad_res;
      // mc_rtc::log::info("grad [{}] = {}", j+6 , grad_res);

      j+=1;
    }
  ctl_.robot().forwardKinematics(mbc);
  return grad;
}


const double Get_In_Position_Task::compute_effective_mass(rbd::MultiBodyConfig q, mc_control::fsm::Controller & ctl_) const
{
  ctl_.robot().forwardKinematics(q);
  auto robot_mb = ctl_.robot().mb();
  // _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, q_test);
  // double delta_psi = 0.01; // rad

  // std::cout << "nrDOF = " << _nrdof << std::endl;
  // std::cout << "size = " << std::size(qf.q) << std::endl;
  // int n = 0;
  // std::cout << "qf = {"  << std::endl;

  // Store only the actuated dofs in Q
  // std::vector<double> Q = {};
  // for (size_t i = 0; i < std::size(qf.q); ++i)
  // {
  //   std::cout << ctl_.robot().mb().joint(i).name()<< " : " ; 
  //   std::cout << "{";
  //   for(size_t j = 0; j < std::size(qf.q.at(i)); ++j)
  //   {
      
  //     std::cout << qf.q.at(i).at(j) << ", ";
  //     n+=1;
  //     Q.push_back(qf.q.at(i).at(j));
  //   }
  //   std::cout << "}" << std::endl;
  // }
  // std::cout << "}" << std::endl;
  // std::cout << "n = " << n << std::endl;

  // std::cout << "Q = {"  << std::endl;
  // for (size_t i = 0; i < std::size(Q); ++i){
  //   std::cout << Q.at(i) << std::endl;
  // }
  // std::cout << "}"  << std::endl;
  Eigen::MatrixXd M = _dynamicsConstraint->motionConstr().fd().H();
    // std::cout << "M  = \n" << M << "\n";
  Eigen::MatrixXd M_inverse = M.inverse();
  // std::cout << "Creating my Jacobian" << std::endl;
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  auto world_frame_jacobian = jac.jacobian(robot_mb,q);
  // std::cout << "Created world frame jacobian" << std::endl;

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl_.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
  // std::cout << "Projected my world frame Jacobian" << std::endl;
  
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);
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
    outer_sum = 0;
    for(int k=0; k <= _nrdof - 1 ; ++k)
    {
      // Inner sum
      inner_sum = 0;
      for(int j=0; j <= _nrdof - 1; ++j)
      { 
        J_nu_3_j = linear_jacobian(3-1,j);
        // std::cout << "Computed linear jac (3-1, "  << j  << ") = " << linear_jacobian(3-1, j) << std::endl;
        M_inverse_j_k = M_inverse(j, k);
        // std::cout << "M_inverse (" << j << "," << k << ") = " << M_inverse_j_k << std::endl;
        inner_sum += J_nu_3_j * M_inverse_j_k;
      }
      J_nu_3_k = linear_jacobian(3-1, k);
      outer_sum += inner_sum*J_nu_3_k;
    }

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
  return 1/outer_sum;
}

// void Get_In_Position_Task::increment_config(rbd::MultiBodyConfig & mbc);

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