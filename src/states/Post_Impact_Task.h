#pragma once

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_control/fsm/State.h>

// BSpline
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_trajectory/BSpline.h>

// Ros node
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <thread>

// ctl
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_trajectory/BSpline.h>
#include <ndcurves/curve_constraint.h>


struct Post_Impact_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:

    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> BSplineVel;
    mc_trajectory::BSpline::waypoints_t posWp;



    typedef Eigen::Vector3d Point;
    typedef Point point_t;
    typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
    curve_constraints_t _constr;

    typedef Eigen::Vector3d vector3_t;
    vector3_t _start_point;
    vector3_t _end_point;
    
    sva::PTransformd _initial_nail_position;    


    Eigen::Matrix<double, 3, 1> _rotation_axis;

    vector3_t _force_vector;
    double _max_force_norm = 0;
    double _max_normal_force_norm = 0;

    vector3_t _peak_force = {0, 0, 1};
    vector3_t _peak_normal_force = {0, 0, 1};
    vector3_t _normal_force = {0, 0, 1};
    double _normal_force_norm = 1;
    bool _height_reached = false;
    sva::PTransformd _initial_hammerhead_position;
    // Ros 
    bool _ros_node_running = false;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _subForce;
    std::thread _ros_thread;


    // ---------------- Parameters -----------------
    mc_rtc::Configuration _config;
    std::string _main_robot_name = "hrp5_p";
    std::string _hammer_head_frame_name = "Hammer_Head";
    
    // qol
    bool _bezier_curve_verbose_active = false;

    // Magic values

    double _magic_max_control_point_height = 1;
    double _magic_bezier_curve_max_duration = 1;
    double _magic_task_stiffness = 1;
    double _magic_task_weight = 1;
    double _magic_epsilon = 1;
    double _magic_oriWp_time = 1;
    double _magic_post_impact_final_height = 1;
    double _magic_coefficient_of_restitution = 1;

    void load_parameters();
    void store_force(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force);

};
