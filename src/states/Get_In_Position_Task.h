#pragma once

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <cstdint>
#include <mc_control/fsm/State.h>

#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_trajectory/BSpline.h>

#include <memory>
#include <ndcurves/curve_constraint.h>
// #include <pinocchio/fwd.hpp>
// #include <pinocchio/multibody/sample-models.hpp>
#include <string>

// Ros node
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <thread>

#include <RBDyn/Jacobian.h>
#include <vector>


struct Get_In_Position_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:

    // Needed to access the mass matrix 
    std::unique_ptr<mc_solver::DynamicsConstraint> _dynamicsConstraint;

    // BSpline curve
    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> BSplineVel;
    sva::PTransformd _initial_hammerhead_position;
    sva::PTransformd _initial_nail_position;    
    mc_trajectory::BSpline::waypoints_t _posWp;
    std::vector<std::pair<double, Eigen::Matrix3d>> _oriWp = {};

    typedef Eigen::Vector3d Point;
    typedef Point point_t;
    typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
    sva::PTransformd _target;
    
    typedef Eigen::Vector3d vector3_t;
    vector3_t _start_point;
    vector3_t _end_point;
    bool stop = false;

    //Position task test
    std::shared_ptr<mc_tasks::PositionTask> _positionTask;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _subForce;


    // Forces
    vector3_t _force_vector;
    double _max_force_norm = 0;
    double _max_normal_force_norm = 0;

    vector3_t _peak_force = {0, 0, 0};
    vector3_t _peak_normal_force = {0, 0, 0};
    Eigen::Matrix<double, 3, 1> _rotation_axis;
    

    bool _impact_detected = false;
    // void store_force(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force);
  
    const double compute_effective_mass(rbd::MultiBodyConfig qf, mc_control::fsm::Controller & ctl_) const;
    const double compute_effective_mass_2(rbd::MultiBodyConfig q, mc_control::fsm::Controller & ctl_, const Eigen::Vector3d &normal_vector) const;
    const Eigen::VectorXd compute_gradient(rbd::MultiBodyConfig q, mc_control::fsm::Controller &ctl_, const Eigen::Vector3d &normal_vector) const;

    void writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) const;
    void printConfig(rbd::MultiBodyConfig q, std::string string) const;
    void writeVectorToCSV(const std::vector<double> & vec, const std::string & filename) const;
    int _nrdof = 41;
    double _effective_mass = 0;
    bool _create_file = true;
    std::vector<double> _masses;

    //finite differences dm/dt
    rbd::MultiBodyConfig _old_mbc;
    double _old_effective_mass = 1;
    const Eigen::Vector3d _normal_vector = {0, 0, 1};
    const double mass_time_derivative(const Eigen::VectorXd &gradient, mc_control::fsm::Controller & ctl_) const;

    // -------------------------------- Parameters ---------------------------------------
    
    // Parameters loaded in the load_parameters function, parameters are found in the Hammering_FSM_Controller.in.yaml file
    // Don't ask me why there is a '.in' in the name of the file, I don't know 
    
    mc_rtc::Configuration _config;
    void load_parameters();
    
    std::string _nail_robot_name = "nail";
    std::string _main_robot_name = "hrp5_p";
    std::string _hammer_head_frame_name = "Hammer_Head";
    std::string _nail_frame_name = "nail";
    
    // timestep
    double _timestep = 1;

    // quality of life
    
    bool _bezier_curve_verbose_active = false;
    bool _jacobian_verbose_active = false;

    
    // gui
    
    std::string _stop_hammering_button_name = "undefined";
    
    //curve constraints
    
    curve_constraints_t _constr;
    
    // Default magic values, just for testing, all loaded in the load_parameters function
    
    double _magic_max_control_point_height = 1;
    double _magic_bezier_curve_max_duration = 1;
    double _magic_task_stiffness = 1;
    double _magic_task_weight = 1;
    double _magic_epsilon = 1;
    double _magic_oriWp_time = 1;
    double _posture_task_weight = 1;
    double _effective_mass_maximization_task_weight = 1;

    
    double _magic_epsilon_force_norm_threshold = 1;

};
