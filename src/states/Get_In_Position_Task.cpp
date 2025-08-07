#include "Get_In_Position_Task.h"

#include "../Hammering_FSM_Controller.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <ostream>
#include <vector>

// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"

// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/cppad.hpp>
// #include <cppad/cppad.hpp>

void Get_In_Position_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
    mc_rtc::log::info("Get_In_Position_Task configure function called with config : \n{}", config.dump(true, true));
    
}

void Get_In_Position_Task::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  load_parameters();
  mc_rtc::log::info("solver timestep = \n{}", ctl.solver().dt());
  _nrdof = ctl.robot().mb().nrDof();
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
  // _end_point = {x_nail_init, y_nail_init, _magic_max_control_point_height};
  _end_point = {x_nail_init, y_nail_init, z_nail_init};

  // Found by calculations by hand
  _rotation_axis = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
  auto angle = M_PI;
  // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
  // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, _rotation_axis));

  _posWp ={_start_point,
          // vector3_t{x_nail_init,y_nail_init, _magic_max_control_point_height},
          _end_point, //arbitrary z for now
          };
  
  
  double bezier_duration = std::abs(_constr.end_vel.z()/_constr.end_acc.z());
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
                                                                  bezier_duration,
                                                                  _magic_task_stiffness, 
                                                                  _magic_task_weight, 
                                                                  _target, 
                                                                  _constr, 
                                                                  _posWp, 
                                                                  _oriWp,
                                                                  _bezier_curve_verbose_active);
  BSplineVel->stiffness(_magic_task_stiffness);
  BSplineVel->weight(_magic_task_weight);
  BSplineVel->selectActiveJoints(
        {
          "LWRR",
        "LWRP",
        "LWRY",
        "LSP",
        "LSR",
        "LSY",
        "LEP",  
      });
  // std::cout << "BSplineVel->spline().get_bezier().constr_.end_vel.z(): " << BSplineVel->spline().get_bezier()->constr_.end_vel.z() << std::endl;
  // std::cout << "end vel z : " << constr.end_vel.z() << std::endl;
  std::cout << "Degree of spline = " << BSplineVel->spline().get_bezier()->degree() << std::endl;
  ctl.getPostureTask(ctl.robot().name())->weight(1);
  ctl.solver().addTask(BSplineVel);
  _dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(ctl.robots(), 
                                                                        ctl.robot(_main_robot_name).robotIndex(),
                                                                      ctl.solver().dt(),
                                                                    true);
  ctl.solver().addConstraintSet(_dynamicsConstraint);
}

bool Get_In_Position_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

    // auto hammer_translation = ctl.robot().frame(_hammer_head_frame_name).position().translation();

    // double position_error = sqrt(std::pow(hammer_translation.x() - _end_point.x(), 2) 
    //                             + std::pow(hammer_translation.y() - _end_point.y(), 2)
    //                            + std::pow(hammer_translation.z() - _magic_max_control_point_height, 2));
    if( BSplineVel->eval().norm() < _magic_epsilon)
    {
      _effective_mass = compute_effective_mass(ctl.robot().mbc(), ctl_);
      std::cout << "Effective mass = " << _effective_mass << std::endl;
      output("STOP");
      return true;
    }
    // if( position_error < _magic_epsilon)
    // {
    //     output("STOP");
    //     return true;
    // }
    // else{
    //   std::cout << "Position error : " << position_error << std::endl;
    // }
  return false;
}

void Get_In_Position_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

  ctl_.gui()->removeElement({}, _stop_hammering_button_name);

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

// void Get_In_Position_Task::store_force(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force)
// {
//     _force_vector = {force -> vector.x, force -> vector.y, force->vector.z};
    
//     vector3_t normal_force = {0, 0, _force_vector.z()};
//     double normal_force_norm = normal_force.norm();

//     if(normal_force_norm > _max_normal_force_norm)
//     {
//         _max_normal_force_norm = normal_force_norm;
//         _peak_normal_force = normal_force;
//     }

//     if (normal_force_norm > _magic_epsilon)
//     {
//         std::cout << "peak force vector : {" << std::endl 
//         << _peak_normal_force.x() << std::endl
//         << _peak_normal_force.y() << std::endl
//         << _peak_normal_force.z() << std::endl
//         << "}" << std::endl;
//     }

// }::

const double Get_In_Position_Task::compute_effective_mass(rbd::MultiBodyConfig qf, mc_control::fsm::Controller & ctl_) const
{

  double delta_psi = 0.01; // rad

  // std::cout << "nrDOF = " << _nrdof << std::endl;
  // std::cout << "size = " << std::size(qf.q) << std::endl;
  // int n = 0;
  // std::cout << "qf = {"  << std::endl;

  // Store only the actuated dofs in Q
  // std::vector<double> Q = {};
  // for (size_t i = 0; i < std::size(qf.q); ++i)
  // {
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
 
  auto robot_mb = ctl_.robot().mb();
  std::cout << "Creating my Jacobian" << std::endl;
  rbd::Jacobian jac(robot_mb, _hammer_head_frame_name, _jacobian_verbose_active);
  auto world_frame_jacobian = jac.jacobian(robot_mb,qf);
  std::cout << "Created world frame jacobian" << std::endl;

  Eigen::MatrixXd full_world_frame_jacobian(6, ctl_.robot().mb().nrDof());
  jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
  std::cout << "Projected my world frame Jacobian" << std::endl;
  
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.topRows(3);
  std::cout << "Created linear jacobian" << std::endl;

  Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);
  std::cout << "Created angular jacobian" << std::endl;

  // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
  Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
  std::cout << "Created pseudo inverse" << std::endl;
  
  auto pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);
  std::cout << "Created pseudo inverse angular jac" << std::endl;

  Eigen::MatrixXd delta_x_omega(3, 1);
  std::cout << "Created delta x omega" << std::endl;

  delta_x_omega << 0, 0, delta_psi;

  Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
  std::cout << "Created delta q" << std::endl;

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
  auto q_test = qf;
  // std::cout << "Created M inverse" << std::endl;
  // std::cout << "Size M^-1 = " << M_inverse.rows() << " x " << M_inverse.cols() << std::endl;
  // writeEigenMatrixToCSV(M_inverse, "InverseMassMatrix.csv");
  // writeEigenMatrixToCSV(linear_jacobian, "LinearJacobian.csv");
  // std::cout << M << std::endl;
  
  
  
  double inner_sum = 0;
  double J_nu_3_j = 0;
  double J_nu_3_k = 0;
  double M_inverse_j_k = 0;
  
  double outer_sum = 0;
  double effective_mass = 0;
  double delta_psi_tot = 0;
  auto q_start = qf;
  std::cout << "Starting double sum" << std::endl;
  
  while(delta_psi_tot < 2*M_PI)
  {
    std::cout << "delta_psi_tot = " << delta_psi_tot << std::endl;
    //Create everything
    ctl_.robot().forwardKinematics(q_test);
    _dynamicsConstraint->motionConstr().fd_non_const().computeH(robot_mb, q_test);
    Eigen::MatrixXd M_inverse = _dynamicsConstraint->motionConstr().fd().H().inverse();
    // std::cout << M_inverse.row(0) << std::endl;
    world_frame_jacobian = jac.jacobian(robot_mb,q_test);

    Eigen::MatrixXd full_world_frame_jacobian(6, robot_mb.nrDof());
    jac.fullJacobian(robot_mb, world_frame_jacobian, full_world_frame_jacobian);
    
    Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

    // Eigen::MatrixXd angular_jacobian = full_world_frame_jacobian.bottomRows(3);

    // std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
    // Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
    
    // pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);

    // Eigen::MatrixXd delta_x_omega(3, 1);

    Eigen::MatrixXd pseudo_inverted_full_jac = full_world_frame_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(full_world_frame_jacobian.rows(), full_world_frame_jacobian.rows()));
    delta_x_omega << 0, 0, delta_psi;
    Eigen::MatrixXd delta_x(6, 1);
    delta_x << 0, 0, 0, 0, 0, delta_psi;
    // Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
    Eigen::MatrixXd delta_q =  pseudo_inverted_full_jac* delta_x;


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

    std::cout << "Testing effective mass : "<< 1/outer_sum << std::endl;
    if(1/outer_sum > effective_mass)
    {
      effective_mass = 1/outer_sum;
    }

    // std::cout << "deltaq =  " << delta_q << std::endl;

    // printConfig(q_test, "q_test before = {");        
    // increment q_test by delta q
    int L = 0;
    int m = 0;
    for (size_t i = 0; i < std::size(q_test.q); ++i)
    {
      for(size_t j = 0; j < std::size(q_test.q.at(i)); ++j)
      {
        q_test.q.at(i).at(j) += delta_q(j+L, 0);
        m += 1;
      }
      L += m;
      m = 0;
    }

    // printConfig(q_test, "q_test after = {");        
    delta_psi_tot += delta_psi;
  }
  return effective_mass;
}

// void Get_In_Position_Task::increment_config(rbd::MultiBodyConfig & mbc);

void Get_In_Position_Task::printConfig(rbd::MultiBodyConfig mbc, std::string string) const
{
    std::cout << string << std::endl;
    for (size_t i = 0; i < std::size(mbc.q); ++i)
    {
      for(size_t j = 0; j < std::size(mbc.q.at(i)); ++j)
      {
        std::cout << mbc.q.at(i).at(j) << std::endl;
      }
    }
    std::cout << "}" << std::endl;
}

void Get_In_Position_Task::writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const {
    std::cout << "entering write eigen function" << std::endl;
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    std::cout << "For loop writeEigen begin" << std::endl;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) file << ",";  // Separate columns by commas
        }
        file << "\n";  // Newline for each row
    }
    file.close();
    std::cout << "Matrix written to " << filename << std::endl;
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