#include "Post_Impact_Task.h"
#include "../Hammering_FSM_Controller.h"
#include <cmath>

void Post_Impact_Task::configure(const mc_rtc::Configuration & config)
{
    _config.load(config);
    mc_rtc::log::info("Post_Impact_Task configure function called with config :  \n{}", config.dump(true, true));
    load_parameters();

    auto nh = mc_rtc::ROSBridge::get_node_handle();
    
    if(nh != nullptr)
    {
        _subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/nail_force_sensor", 
            1000,
            std::bind(&Post_Impact_Task::store_force, this, std::placeholders::_1));   

        mc_rtc::log::success("Post_Impact_Task.cpp initialized");
    }
    else
    {
        mc_rtc::log::error("Post_Impact_Task.cpp configure function : nh is nullptr");
    }
}

void Post_Impact_Task::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<Hammering_FSM_Controller &>(ctl_);
    _initial_hammerhead_position = ctl.robot().frame(_hammer_head_frame_name).position();
    // _start_point = _initial_hammerhead_position.translation();
    
    // _end_point = {_start_point.x(), _start_point.y(), _magic_post_impact_final_height};
    
    _initial_nail_position = ctl.robots().robot("nail").frame("nail").position();
    auto initial_nail_translation = _initial_nail_position.translation();
    _start_point = {initial_nail_translation.x(), initial_nail_translation.y(), initial_nail_translation.z() + 0.3};
    _end_point = {initial_nail_translation.x(), initial_nail_translation.y(), _magic_post_impact_final_height};

    _constr.end_vel.z() = _magic_coefficient_of_restitution * 1;

//  create a simple bspline that has the same direction as the normal force 
//   ctl.getPostureTask(ctl.robot().name())->weight(1);
    posWp ={_start_point,
            _end_point,                                    
            };

  // Found by calculations by hand
    _rotation_axis = Eigen::Matrix<double, 3, 1>(1,0,-1).normalized();
    auto angle = M_PI;
    // The quaternion works for a nail placed on a horizontal table, it will not work for a nail placed on a slope
    // The angle and the rotation axis should be computed for different orientations of the nail, but I did not do it
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, _rotation_axis));
    // auto rotation_matrix = q.toRotationMatrix();
    // std::cout << "q.toRotationMatrix() = " << q.toRotationMatrix() << std::endl; 
    // double roll = atan2(rotation_matrix(3, 2), rotation_matrix(3, 3));
    // double pitch = atan2(-rotation_matrix(3, 1), sqrt(std::pow(rotation_matrix(3, 2), 2) + std::pow(rotation_matrix(3, 3),2)  )); 
    // std::cout << "roll = " << roll << std::endl;
    // std::cout << "pitch = " << pitch << std::endl;

    const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {
        // std::make_pair(_magic_oriWp_time, q.matrix()) // at t = _magic_oriWp_time, arbitrary for now

    };

    const sva::PTransformd & target = sva::PTransformd(q, //maybe the quaternion expresses the orientation of your rotating frame you want to achieve at the end with respect to the world frame
                                                            //thus whatever the starting orientation of the rotating frame wrt the world frame, the robot frame will try to end up at the orientation specified by the quaternion
                                                            //how does it do that ?
                                                  _end_point);
  
    BSplineVel = std::make_shared<mc_tasks::BSplineTrajectoryTask>(ctl.robot().frame(_hammer_head_frame_name),
                                                                  _magic_bezier_curve_max_duration, 
                                                                  _magic_task_stiffness, 
                                                                  _magic_task_weight, 
                                                                  target, 
                                                                  _constr, 
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

bool Post_Impact_Task::run(mc_control::fsm::Controller & ctl)
{
    // _height_reached = BSplineVel->eval().norm() < _magic_epsilon;
    auto hammer_head_translation = ctl.robot().frame(_hammer_head_frame_name).position().translation();
    _height_reached = sqrt(pow(hammer_head_translation.z() - _magic_post_impact_final_height, 2)) < _magic_epsilon;

    if(_height_reached)
    {
        output("HAMMERING_HEIGHT_REACHED");
        return true;
    }
    else{
        // std::cout << "BSplineVel->eval().norm() = " << BSplineVel->eval().norm() << " >= " << _magic_epsilon << std::endl;
    }
    return false;
}

void Post_Impact_Task::teardown(mc_control::fsm::Controller & ctl)
{
//   ctl.getPostureTask(ctl.robot().name())->weight(10);
}

void Post_Impact_Task::load_parameters()
{
    _main_robot_name = "hrp5_p";

    // ------------------------ Loading quality of life parameters ---------------------------

    std::string qol_key = "quality_of_life";
    std::string bezier_curve_verbose_active_key = "bezier_curve_verbose_active";
    _bezier_curve_verbose_active = _config(qol_key)(bezier_curve_verbose_active_key);

    // ------------------------ Loading frames ---------------------------

    std::string frames_key = "frames";
    std::string hammerhead_frame_key = "Hammer_Head";
    _config(frames_key)(hammerhead_frame_key, _hammer_head_frame_name);

    // ------------------------ Loading magic values ---------------------------

    std::string magic_values_key = "magic_values";
    _magic_epsilon = _config(magic_values_key)("epsilon");
    _magic_bezier_curve_max_duration = _config(magic_values_key)("bezier_curve_max_duration");
    _magic_task_stiffness = _config(magic_values_key)("task_stiffness");
    _magic_task_weight = _config(magic_values_key)("task_weight");
    _magic_post_impact_final_height = _config(magic_values_key)("post_impact_final_height");
    _magic_coefficient_of_restitution = _config(magic_values_key)("coefficient_of_restitution");

    // ------------------------ Loading curve constraints ---------------------------
    std::string curve_constraints_key = "curve_constraints";
    std::string linear_velocity_key = "linear_velocity";
    std::string z_key = "z";
    std::string init_key = "init";
    _constr.end_vel.z() = _config(curve_constraints_key)(linear_velocity_key)(z_key)(init_key);


}

void Post_Impact_Task::store_force(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force)
{
    // Is it possible to directly get the value of the peak force from the previous task ?
    _force_vector = {force -> vector.x, force -> vector.y, force->vector.z};
    
    _normal_force = {0, 0, _force_vector.z()};
    _normal_force_norm = _normal_force.norm();

    if(_normal_force_norm > _max_normal_force_norm)
    {
        _max_normal_force_norm = _normal_force_norm;
        _peak_normal_force = _normal_force;
    }

    if (_normal_force_norm > _magic_epsilon)
    {
        std::cout << "peak force vector : {" << std::endl 
        << _peak_normal_force.x() << std::endl
        << _peak_normal_force.y() << std::endl
        << _peak_normal_force.z() << std::endl
        << "}" << std::endl;
    }

}

EXPORT_SINGLE_STATE("Post_Impact_Task", Post_Impact_Task)
