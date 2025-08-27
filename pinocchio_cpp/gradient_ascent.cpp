#include <cmath>
#include <cstddef>
#include <iterator>
#include <ndcurves/bezier_curve.h>
#include <ndcurves/curve_constraint.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace pinocchio;
using namespace Eigen;

// Load URDF model and initialize global variables
const std::string model_path = "/home/jimmyvu/Documents/mc_rtc/src/catkin_data_ws/src/hrp5_p_description/urdf/HRP5Pmain.urdf";
Model model;
Data data(model);
FrameIndex frame_id;
Vector3d normal_vector(0, 0, 1);


void writeEigenMatrixToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) {
    std::cout << "entering write eigen function" << std::endl;
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
    std::cout << "Matrix written to " << filename << std::endl;
}
// Compute effective mass in a direction
double compute_effective_mass(const Model &model, const VectorXd &q, const Vector3d &normal_vector) {
    Data data(model);
    normalize(model, q);
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    computeJointJacobians(model, data, q);

    // Pinocchio stores only the upper triangular part of the mass matrix
    crba(model, data, q);
    
    // Symetrize the mass matrix
    data.M.triangularView<StrictlyLower>() = data.M.transpose().triangularView<StrictlyLower>();

    // computeFrameJacobian(model, data, q, frame_id, LOCAL_WORLD_ALIGNED, fullJ);
    // writeEigenMatrixToCSV(fullJ, "fullJacobian.csv");
    MatrixXd fullJ(6, model.nv); 
    fullJ.setZero();
    std::string frame_name = "Hammer_Head";
    // frame_id = model.getFrameId(frame_name);
    int frame_id = model.getFrameId(frame_name, pinocchio::BODY);
    pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::WORLD, data.J);
    auto linearJ = data.J.topRows(3);
    writeEigenMatrixToCSV(data.J, "FullJacobianPinocchio.csv");
    writeEigenMatrixToCSV(data.M, "MassMatrixPinocchio.csv");

    // std::cout << fullJ << std::endl;
    MatrixXd Minv = data.M.inverse();
    // std::cout << data.M << std::endl;
    // std::cout << Minv.rows() << " x " << Minv.cols() << std::endl;
    // std::cout << linearJ.rows() << " x " << linearJ.cols() << std::endl;

    MatrixXd Lambda = linearJ * Minv * linearJ.transpose();
    // std::cout << Lambda << std::endl;
    double effective_mass = 1 / (normal_vector.transpose() * Lambda * normal_vector);
    // std::cout << "Product done" << std::endl;
    return effective_mass;
}

// Numerical gradient of effective mass
VectorXd gradient_effective_mass(const Model &model, const VectorXd &q, const Vector3d &normal_vector, double epsilon = 1e-3) {
    VectorXd grad = VectorXd::Zero(q.size());

    for (int i = 0; i < q.size(); ++i) {
        VectorXd dq = VectorXd::Zero(q.size());
        dq[i] = epsilon;
        // std::cout << "i = "<< i << std::endl;
        double f_plus = compute_effective_mass(model, q + dq, normal_vector);
        double f_minus = compute_effective_mass(model, q - dq, normal_vector);

        grad[i] = (f_plus - f_minus) / (2.0 * epsilon);
    }
    return grad;
}

double right_integral(const std::vector<double> &dxs, const double &dt){
    double x_right_integral = 0;
    int i = 0;
    for(const double & value: dxs)
    {
        // std::cout << "value : " << value << std::endl;
        if (i==0)
        {
            i+=1;
            continue;
        }
        x_right_integral += value*dt;
        
    }
    return x_right_integral;
}

double left_integral(const std::vector<double> &dxs, const double &dt){
    double x_left_integral = 0;
    int i = 0;
    for(const double & value: dxs)
    {
        // std::cout << "value : " << value << std::endl;
        if (i==std::size(dxs)-1)
        {
            
            return x_left_integral;
        }
        x_left_integral += value*dt;
        i+=1;
    }
}
Eigen::MatrixXd U_Matrix(const double &t, const double &T, const double &t_min)
{
    Eigen::MatrixXd res(1, 4);
    res << 
    std::pow((t-t_min)/T, 3),
    std::pow((t-t_min)/T, 2),
    (t-t_min)/T,
    1;
    return res;
}

Eigen::MatrixXd U5_Matrix(const double &t, const double &T, const double &t_min)
{
    Eigen::MatrixXd res(1, 6);
    res << 
    std::pow((t-t_min)/T, 5),
    std::pow((t-t_min)/T, 4),
    std::pow((t-t_min)/T, 3),
    std::pow((t-t_min)/T, 2),
    (t-t_min)/T,
    1;
    return res;
}

Eigen::MatrixXd A_Matrix()
{
    Eigen::MatrixXd A(4,4);
    A << 
    -1, 3, -3, 1,
    3, -6, 3, 0,
    -3, 3, 0, 0,
    1, 0, 0, 0;
    return A;
}

Eigen::MatrixXd A5_Matrix()
{
    Eigen::MatrixXd A(6,6);
    A << 
    -1, 5, -10, 10, -5, 1,
    5, -20, 30, -20, 5, 0,
    -10, 30, -30, 10, 0, 0,
    10, -20, 10, 0, 0, 0,
    -5, 5, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0
    ;
    return A;
}


// Gradient ascent
Eigen::VectorXd gradient_ascent_effective_mass(const Eigen::VectorXd &q,
                                                const Eigen::Vector3d& normal_vector,
                                                double epsilon,
                                                int iterations,
                                                double eta)
{
    const double threshold = 0.01;
    auto q_res = q;
    for (int i = 0; i < iterations; ++i)
    {
        Eigen::VectorXd grad = gradient_effective_mass(model, q_res, normal_vector, epsilon);
        Eigen::VectorXd step = eta * grad;

        if (step.norm() < threshold)
        {
            break;
        }

        q_res += step; // Update joint configuration
    }

    return q_res;
}
double trapezoidal_integral(const std::vector<double> &dxs, const double &dt)
{
    return (right_integral(dxs, dt) + left_integral(dxs, dt)) / 2;
}
int main() {
std::vector<double> q_init = {
-8.0696e-05,
0.000139596,
-0.469204,
0.874495,
-0.405169,
-0.0001583,
-9.21395e-05,
0.000108508,
-0.468936,
0.87441,
-0.405337,
4.2664e-05,
-0.000151574,
-0.000334872,
0.00458754,
-0.00235967,
-0.0133775,
-0.000115615,
1.04722,
0.348996,
0.0869743,
-1.83261,
0.000191748,
0.697881,
0.000123788,
0.000190558,
0.0109295,
1.04689,
-0.349471,
-0.0873123,
-1.83271,
1.00752e-05,
-0.698116,
-1.11371e-05,
-9.09193e-07,

};
// std::vector<double> q_init = {
// -0.659376, 
// -15.0396,
// -5.16333, 
// -78.6495, 
// -4.50001, 
// -5.34048, 
// -0.731255, 
// 14.5138, 
// -0.140389, 
// -74.6469,
// -3.85331, 
// 4.96727, 
// 35.9288, 
// -0.164685, 
// 1.8159, 
// -0.381541, 
// -0.565998, 
// -0.0476309, 
// 4.64745, 
// 6.21565, 
// 6.23047,
// -13.7554, 
// 0.28222, 
// 1.7844, 
// -4.43023, 
// 0.814406, 
// 2.16581, 
// 6.65938, 
// -7.91135, 
// -5.83129, 
// -9.22931,
// 0.767996, 
// -1.34104, 
// -2.09051, 
// 0.0396422, 
// };


    // Load the URDF model
    std::vector<std::string> jointsToRemove = {
        // " azure_depth_frame_joint",
        // "head_root_joint",
        // "left_camera_optical_frame_joint",
        // "left_camera_optical_joint",
        "motor_joint",
        // "hokuyo_link_joint",
        // "head_hokuyo_frame_joint",
        // "right_camera_optical_frame_joint",
        // "realsense_link_joint",
        // "realsense_aligned_depth_to_color_frame_joint",
        // "realsense_color_optical_joint",
        // "realsense_color_frame_joint", 
        // "realsense_depth_joint",
        // "realsense_depth_optical_joint",
        // "hammer_root",
        // "hammer_head",
        "LIMP",
        "LIPIP",
        "LIDIP",
        "LMMP",
        "LMPIP",
        "LMDIP",
        "LTMP",
        "LTPIP",
        "LTDIP",
        "RIMP",
        "RIPIP", 
        "RIDIP", 
        "RMMP", 
        "RMPIP", 
        "RMDIP",
        "RTMP", 
        "RTPIP", 
        "RTDIP", 
    };
    model = pinocchio::urdf::buildModel(model_path, model);
    std::vector<JointIndex> joints_to_lock_ids;
    for(const auto &joint_name: jointsToRemove)
    {
        std::cout << joint_name << "Id = " << model.getJointId(joint_name) << "\n"; 
        joints_to_lock_ids.push_back(model.getJointId(joint_name));
    }

    // Initial configuration
    VectorXd q = neutral(model);
    std::cout << "Model loaded: " << model.name << "\n";
    data = Data(model);

    Model reduced_model = buildReducedModel(model, joints_to_lock_ids, q);
    pinocchio::Data reduced_data(reduced_model);
    VectorXd q_reduced = neutral(reduced_model);
    
    // Print joint info
    for (JointIndex i = 0; i < reduced_model.joints.size(); ++i) {
        const auto &joint = reduced_model.joints[i];
        std::cout << "Joint " << i << ": " << reduced_model.names[i] << "\n";
                //   << " (type: " << joint.shortname() << ", dof: " << joint.nv() << ")\n";
    }

    // Get frame index
    // std::string frame_name = "Hammer_Head";
    // frame_id = model.getFrameId(frame_name);
    // if (frame_id == model.nframes) {
    //     std::cerr << "Frame '" << frame_name << "' not found.\n";
    //     return 1;
    // }
    
    for (size_t i = 0; i < std::size(q_init); ++i)
    {
        q_reduced(i) = q_init.at(i);
    }
    pinocchio::forwardKinematics(reduced_model, reduced_data, q_reduced);
    pinocchio::updateFramePlacements(reduced_model, reduced_data);
    
    // Compute gradient of effective mass
    // VectorXd grad = gradient_effective_mass(reduced_model, q_reduced, normal_vector);
    // std::cout << "Gradient:\n" << grad << "\n";
    

    // SE3 frame_placement = reduced_data.oMf[frame_id];
    // const Frame & frame = reduced_model.frames[frame_id];
    // pinocchio::SE3 parent_pose = data.oMi[frame.parentJoint]; // pose of parent joint in world frame
    // pinocchio::SE3 frame_pose = parent_pose * frame.placement; 
    // std::cout << "parent frame : " << frame.parentJoint << std::endl;
    // std::cout << "frame pose : " << frame_pose << std::endl;

    std::cout << "Effective mass at q_neutral: " << compute_effective_mass(reduced_model, q_reduced, normal_vector) << " kg \n";
    // Eigen::Matrix3d rotation_matrix = frame_placement.rotation();
    // std::cout << "Rotation matrix of frame '" << frame_name << "':\n"
    // << rotation_matrix << std::endl;
    
    // Trajectory
    
    //Starting point
    double x = 0.3229712491149373;
    double y = 0.257625;
    double z = 0.92239;
    Eigen::Vector3d starting_point = {x, y, z};
    Eigen::Vector3d nail_point = {0.55, 0.30, 0.9};
    std::vector<Eigen::Vector3d> control_points = {
        starting_point,
        nail_point
    };

    double T_min = 0.0;
    double T_max = 1.0;
    typedef Eigen::Vector3d Point;
    typedef Point Point_t;
    ndcurves::curve_constraints<Point_t> constraints;

    constraints.end_vel.z() = -1;
    constraints.end_acc.z() = 0;
    constraints.end_jerk.z() = 0;

    ndcurves::bezier_curve bezier(control_points.begin(), 
    control_points.end(), 
    constraints,
    T_min, 
    T_max);
    //Sample points
    std::vector<double> ts, xs, ys, zs, dxs, ddxs, dys, ddys, dzs, ddzs;
    auto dt = 0.002;
    for (double t = T_min; t <= T_max; t += dt) {
        Eigen::Vector3d p = bezier(t);
        Eigen::Vector3d ppast = bezier(t-dt);
        Eigen::Vector3d pfuture = bezier(t+dt);
        ts.push_back(t);
        xs.push_back(p.x());
        ys.push_back(p.y());
        zs.push_back(p.z());
        if(t == T_min)
        {
            dxs.push_back((pfuture.x() - p.x()) / (dt));
            dys.push_back((pfuture.y() - p.y()) / (dt));
            dzs.push_back((pfuture.z() - p.z()) / (dt));
        }
        else if(t > T_min && t < T_max){
            dxs.push_back((pfuture.x() - ppast.x()) / (2*dt));
            dzs.push_back((pfuture.y() - ppast.y()) / (2*dt));
            dzs.push_back((pfuture.z() - ppast.z()) / (2*dt));

        }
        else {
            dxs.push_back((p.x() - ppast.x())/(dt));
            dzs.push_back((p.y() - ppast.y())/(dt));
            dzs.push_back((p.z() - ppast.z())/(dt));
        }
        // ddxs.push_back((-2*p.x() + pfuture.x() +ppast.x())/(dt*dt));
    }

    std::cout << "trapezoidal integral dxs = " << trapezoidal_integral(dxs, dt) + xs.at(0) << "\n";
    std::cout << "trapezoidal integral dys = " << trapezoidal_integral(dys, dt) + ys.at(0) << "\n";
    std::cout << "trapezoidal integral dzs = " << trapezoidal_integral(dzs, dt) + zs.at(0) << "\n";

    Eigen::Vector3d x_f_linear = {
        trapezoidal_integral(dxs, dt) + xs.at(0),
        trapezoidal_integral(dys, dt) + ys.at(0),
        trapezoidal_integral(dzs, dt) + zs.at(0)
    };
    std::vector<std::vector<double>> x_dot_linear = {
        dxs,
        dys,
        dzs
    };
    // std::cout << "bezier (" << 0.4 << ") = " << bezier(0.4) << std::endl;
    // Eigen::MatrixXd U = U_Matrix(0, 1.0, 0.0);
    // std::cout << "U size = "<< U.rows() << " x " << U.cols() << std::endl;
    // std::cout << "U = "<< U << std::endl;

    // Eigen::MatrixXd A = A_Matrix();
    // std::cout << "A size = "<< A.rows() << " x " << A.cols() << std::endl;
    // std::cout << "A = " << A  << std::endl;

    // Eigen::MatrixXd B = (U*A).transpose();
    // std::cout << "B size = "<< B.rows() << " x " << B.cols() << std::endl;
    // std::cout << "B = " << B  << std::endl;

    // Eigen::MatrixXd Gamma = bezier(0);
    // std::cout << "Gamma transpose size = "<< Gamma.transpose().rows() << " x " << Gamma.transpose().cols() << std::endl;
    // std::cout << "Gamma transpose = " << Gamma.transpose()  << std::endl;

    // Eigen::MatrixXd P = B.completeOrthogonalDecomposition().pseudoInverse()*Gamma;
    Eigen::MatrixXd P(3, std::size(bezier.waypoints()));
    auto startPoint = bezier.waypoints().at(0);
    auto endPoint = bezier.waypoints().at(std::size(bezier.waypoints())-1);
    // std::cout << "bezier_size = "<< std::size(bezier.waypoints()) << std::endl;
    // bool startPointAdded = false;
    // bool endPointAdded = false;

    int i = 0;
    for (const auto &point : bezier.waypoints())
    { 
        P(0,i) = point.x();
        P(1, i) = point.y();
        P(2, i) = point.z();
        i+=1;
    }

    double t_test = 0.4;
    std::cout << "P = \n"<< P.transpose() << std::endl;
    Eigen::MatrixXd Gamma_test = U5_Matrix(t_test, 1.0, 0.0)*A5_Matrix()*P.transpose();
    // std::cout << "Gamma_test at t = " << t_test  << " : " << Gamma_test << std::endl;
    // std::cout << "bezier at t =  "<< t_test << " : " << bezier(t_test) << std::endl;
    // std::cout << bezier(t_test)*(U5_Matrix(t_test, 1.0, 0.0)*A5_Matrix()).completeOrthogonalDecomposition().pseudoInverse().transpose() << std::endl;

    Eigen::MatrixXd B5star(6, 6);
    Eigen::MatrixXd Wp(6, 3);
    Wp << 
    bezier(T_min).transpose(), 
    bezier(0.2).transpose(),
    bezier(0.3).transpose(), 
    bezier(0.55).transpose(),
    bezier(0.6).transpose(), 
    bezier(T_max).transpose();
    B5star << 
    U5_Matrix(T_min, T_max, T_min)*A5_Matrix(),
    U5_Matrix(0.2, T_max, T_min)*A5_Matrix(),
    U5_Matrix(0.3, T_max, T_min)*A5_Matrix(),
    U5_Matrix(0.55, T_max, T_min)*A5_Matrix(),
    U5_Matrix(0.6, T_max, T_min)*A5_Matrix(),
    U5_Matrix(T_max, T_max, T_min)*A5_Matrix();
    
    std::cout << "Computed control points = \n" << B5star.colPivHouseholderQr().solve(Wp)<< std::endl;
    std::cout << "Waypoints : \n" << Wp << std::endl;
    // std::cout << x_f_linear << std::endl;
    // std::cout << "q = {" << "\n";
    // for (const auto &qi : q_reduced)
    // {
    //     std::cout << qi  << std::endl;
    // }
    // std::cout << "}" << std::endl;


    return 0;
}
