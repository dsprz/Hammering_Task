#include "Hit_Task.h"

#include "../Hammering_FSM_Controller.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <RBDyn/MultiBodyConfig.h>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>
#include <mc_solver/DynamicsConstraint.h>
#include <memory>
#include <ostream>

void Hit_Task::configure(const mc_rtc::Configuration & config)
{
   _config.load(config);
  
   mc_rtc::log::info("Hit_Task configure function called with config : \n{}", config.dump(true, true));
   if(_subForce != nullptr)
   {
    _subForce.reset();
    _subForce = nullptr;
   }
   
    _nh = mc_rtc::ROSBridge::get_node_handle();
    if(_nh != nullptr)
    {
        mc_rtc::log::info("Before sub creation");

        _subForce = _nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/nail_force_sensor", 
            1000,
            std::bind(&Hit_Task::store_force, this, std::placeholders::_1));   

        mc_rtc::log::success("Hit_Task.cpp initialized");
    }
    else
    {
        mc_rtc::log::error("Hit_Task.cpp configure function : nh is nullptr");
    }

}

void Hit_Task::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    load_parameters();
    _robot_multibody = ctl.robot().mb();
    _robot_initial_configuration = ctl.robot().q();
    _nrdof = _robot_multibody.nrDof();
    // Get the initial conditions
    _initial_hammerhead_position = ctl.robot().frame(_hammer_head_frame_name).position();
    auto initial_hammerhead_translation = _initial_hammerhead_position.translation();
    
    _initial_nail_position = ctl.robots().robot(_nail_robot_name).frame(_nail_frame_name).position();
    auto initial_nail_translation = _initial_nail_position.translation();
    
    _start_point = initial_hammerhead_translation;
    // _start_point = {initial_nail_translation.x(), initial_nail_translation.y(), initial_hammerhead_translation.z()};

    
    //Using start point because otherwise the trajectory would do weird things because the hammer will not exactly
    // be at the xnail and ynail position Get_In_Position_Task is finished
    // _end_point = {_start_point.x(), _start_point.y(), initial_nail_translation.z()};
    _end_point = initial_nail_translation;
    
    
    // Found by calculations by hand
    _rotation_axis = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
    auto angle = M_PI;
    // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
    // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, _rotation_axis));
    
    _posWp ={_start_point,
      _end_point,                                    
    };
    
    _oriWp = {std::make_pair(4, q.matrix())
    };
    _target = sva::PTransformd(q, 
      _end_point);
      
      // See if the task is tracked well. If it is tracked well but there is still a problem, then it might come from the trajectory.
      // If the trajectory is good but the tracking says otherwise, then there might be a problem with the tracking
      // Check if the hammer mass was correctly taken into account
      BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(_hammer_head_frame_name),
      _magic_bezier_curve_max_duration, 
      _magic_task_stiffness, 
      _magic_task_weight, 
      _target, 
      _constr, 
      _posWp, 
      _oriWp,
      _bezier_curve_verbose_active);
      std::cout << "Degree of spline = " << BSplineVel->spline().get_bezier()->degree() << std::endl;
      BSplineVel->stiffness(_magic_task_stiffness);
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
      BSplineVel->weight(_magic_task_weight);

      
      ctl.getPostureTask(ctl.robot().name())->weight(1);
      ctl.solver().addTask(BSplineVel);
    }

bool Hit_Task::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);

    _impact_detected = _force_vector_on_nail.norm() >= _magic_epsilon_force_norm_threshold;
    if(_impact_detected)
    {
      _final_config = ctl.robot().mbc();
      compute_effective_mass(_final_config, ctl);
      
      output("IMPACT_DETECTED");
      return true;
    }
  
  return false;
}

void Hit_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
  ctl_.gui()->removeElement({}, _stop_hammering_button_name);

  ctl.solver().removeTask(BSplineVel);
  ctl.getPostureTask(ctl.robot().name())->weight(10);
}

void Hit_Task::store_force(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force)
{
    _force_vector_on_nail = {force -> vector.x, force -> vector.y, force->vector.z};
    
    vector3_t normal_force = {0, 0, _force_vector_on_nail.z()};
    double normal_force_norm = normal_force.norm();

    if(normal_force_norm > _max_normal_force_norm_on_nail)
    {
        _max_normal_force_norm_on_nail = normal_force_norm;
        _peak_normal_force_on_nail = normal_force;
    }

    if (normal_force_norm > _magic_epsilon)
    {
        std::cout << "peak force vector : {" << std::endl 
        << _peak_normal_force_on_nail.x() << std::endl
        << _peak_normal_force_on_nail.y() << std::endl
        << _peak_normal_force_on_nail.z() << std::endl
        << "}" << std::endl;
    }

}
const double Hit_Task::compute_effective_mass(rbd::MultiBodyConfig qf, mc_control::fsm::Controller & ctl_) const
{
  std::cout << "nrDOF = " << _nrdof << std::endl;
  std::cout << "size = " << std::size(qf.q) << std::endl;
  std::cout << "qf = {"  << std::endl;
  for (size_t i = 0; i < std::size(qf.q); ++i)
  {
    std::cout << "{";
    for(size_t j = 0; j < std::size(qf.q.at(i)); ++j)
    {
      std::cout << qf.q.at(i).at(j) << ", ";
    }
    std::cout << "}" << std::endl;
  }
  std::cout << "}" << std::endl;

  double delta_psi = 0.1; // rad
  rbd::Jacobian jac(ctl_.robot().mb(), _hammer_head_frame_name);
  auto world_frame_jacobian = jac.jacobian(ctl_.robot().mb(),ctl_.robot().mbc());
  Eigen::MatrixXd linear_jacobian = world_frame_jacobian.topRows(3);
  Eigen::MatrixXd angular_jacobian = world_frame_jacobian.bottomRows(3);
  std::cout << "dim angular jacobian = " << angular_jacobian.rows() << " x " << angular_jacobian.cols() << std::endl;
  Eigen::MatrixXd pseudo_inverted_angular_jacobian = angular_jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(angular_jacobian.rows(), angular_jacobian.rows()));
  
  auto pseudo_inverted_angular_jacobian_third_column = pseudo_inverted_angular_jacobian.col(3 - 1);
  Eigen::MatrixXd delta_x_omega(3, 1);
  delta_x_omega << 0, 0, delta_psi;

  Eigen::MatrixXd delta_q =  pseudo_inverted_angular_jacobian * delta_x_omega;
  std::cout << "delta q = " << delta_q << std::endl;

  // auto incremented_robot_config = qf.q + delta_q;
  // incremented_robot_config.jointConfig;


  double inner_sum = 0;
  double J_nu_n_j = 0;
  double J_nu_3_k = 0;
  double M_inverse_j_k = 0;
  double outer_sum = 1;
  // for(int k=0; k <= _nrdof - 1 ; ++k)
  // {
  //   for(int j=0; j <= _nrdof - 1; ++j)
  //   { 
  //     J_nu_n_j = linear_jacobian(_nrdof - 1, j);
      
  //     _dynamicsConstraint->motionConstr().fd_non_const().computeH(_robot_multibody, qf);
  //     M_inverse_j_k = _dynamicsConstraint->motionConstr().fd().H().inverse()(j, k);

  //     inner_sum += J_nu_n_j * M_inverse_j_k;
  //   }
  //   J_nu_3_k = linear_jacobian(3-1, k);
  //   inner_sum = inner_sum*J_nu_3_k;
  //   outer_sum += inner_sum;
  // }
  double effective_mass = 1/outer_sum;
  return effective_mass;
}

void Hit_Task::load_parameters()
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

//   _constr.end_jerk.x() = _config(curve_constraints_key)(linear_jerk_key)(x_key)(end_key);
//   _constr.end_jerk.y() = _config(curve_constraints_key)(linear_jerk_key)(y_key)(end_key);
//   _constr.end_jerk.z() = _config(curve_constraints_key)(linear_jerk_key)(z_key)(end_key);
}

EXPORT_SINGLE_STATE("Hit_Task", Hit_Task)
