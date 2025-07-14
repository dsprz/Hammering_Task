#pragma once

#include <mc_control/fsm/State.h>

#include <Eigen/src/Geometry/Quaternion.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/MotionVec.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_trajectory/BSpline.h>

#include <ndcurves/curve_constraint.h>
#include <string>



struct Get_In_Position_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  private:

    std::shared_ptr<mc_tasks::BSplineTrajectoryTask> BSplineVel;
    sva::PTransformd _initial_hammerhead_position;
    sva::PTransformd _initial_nail_position;    

    bool stop = false;

    mc_trajectory::BSpline::waypoints_t posWp;

    typedef Eigen::Vector3d Point;
    typedef Point point_t;
    typedef ndcurves::curve_constraints<point_t> curve_constraints_t;
    
    typedef Eigen::Vector3d vector3_t;
    vector3_t _start_point;
    vector3_t _end_point;

    // -------------------------------- Parameters ---------------------------------------

    // Parameters loaded in the load_parameters function, parameters are found in the Hammering_FSM_Controller.in.yaml file
    // Don't ask me why there is a '.in' in the name of the file, I don't know 
    
    mc_rtc::Configuration _config;
    void load_parameters();

    std::string _nail_robot_name = "nail";
    std::string _main_robot_name = "hrp5_p";
    std::string _task_name = "Hammer::Get_In_Position_Task";
    std::string _hammer_head_frame_name = "Hammer_Head";
    std::string _nail_frame_name = "nail";

    // quality of life

    bool _bezier_curve_verbose_active = false;

    // gui

    std::string _stop_hammering_button_name = "undefined";
    
    //curve constraints

    curve_constraints_t constr;

    // Default magic values, just for testing, all loaded in the load_parameters function
    
    double _magic_max_control_point_height = 1;
    double _magic_bezier_curve_max_duration = 1;
    double _magic_task_stiffness = 1;
    double _magic_task_weight = 1;
    double _magic_epsilon = 1;
    double _magic_oriWp_time = 1;


    Eigen::Matrix<double, 3, 1> rotation_axis_;

};
