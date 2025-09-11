#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ostream>
#include <iostream>


const Eigen::Vector3d configuration(const double &time)
{
    Eigen::Vector3d qres;
    qres << time*time, exp(time), time;
    return qres;
}
const Eigen::Vector3d configuration_dot(const double &time)
{
    Eigen::Vector3d q_dot_res;
    q_dot_res << 2*time, exp(time), 1;
    return q_dot_res;
} 
const Eigen::MatrixXd linear_jacobian(const Eigen::Vector3d &q)
{
    const unsigned int ndof = 3;

    Eigen::MatrixXd jac(3,ndof);
    double jac11 = 0;
    double jac12 = exp(q(0, 0));
    double jac13 = q(0, 0);
    double jac21 = 0;
    double jac22 = 2*cos(q(1, 0));
    double jac23 = q(1, 0);
    double jac31 = 0;
    double jac32 = q(2, 0)-0.1;
    double jac33 = 0;
    jac << jac11, jac12, jac13, 
            jac21, jac22, jac23,
            jac31, jac32, jac33;
    return jac;
}
const Eigen::MatrixXd mass_matrix(const Eigen::Vector3d &q)
{
    const unsigned int ndof = 3;
    Eigen::MatrixXd M(ndof,ndof);
    double M11 = 10*cos(q(0, 0));
    double M12 = 5*sin(q(0, 0));
    double M13 = 2*q(0, 0);
    double M21 = M12;
    double M22 = q(1, 0);
    double M23 = 3*cos(q(1, 0));
    double M31  = M13;
    double M32 = M23;
    double M33 = 3*sin(q(2, 0));
    M << M11, M12, M13,
         M21, M22, M23,
         M31, M32, M33;
    return M;
}

const double effective_mass(const Eigen::Vector3d &q, 
                            const Eigen::Vector3d &normal_vector)
{
    // std::cout << "q = " << q << std::endl;
    const Eigen::Matrix3d J_nu = linear_jacobian(q);
    const Eigen::Matrix3d M = mass_matrix(q);
    const Eigen::Matrix3d LAMBDA = J_nu*M.inverse()*J_nu.transpose();
    return 1/(normal_vector.transpose()*LAMBDA*normal_vector);
}

const Eigen::Vector3d emass_grad_central_differences(const double &emass, 
                                const double &time,
                                const Eigen::Vector3d &old_q,
                                const Eigen::Vector3d &normal_vector)
{
    double dqi = 0;
    Eigen::Vector3d grad_res;
    Eigen::Vector3d q = configuration(time);
    double forward_emass = 0;
    double backward_emass = 0;
    
    for(size_t i = 0; i < grad_res.rows(); ++i)
    {
        dqi = q(i, 0) - old_q(i, 0);
        // std::cout << "dqi = " << dqi << std::endl;
        // dqi = 1E-10;
        Eigen::Vector3d forward_q = q;
        Eigen::Vector3d backward_q = q;
        forward_q(i, 0) += dqi;
        backward_q(i, 0) -= dqi;
        forward_emass = effective_mass(forward_q, normal_vector);
        backward_emass = effective_mass(backward_q, normal_vector);
        grad_res(i, 0) = (forward_emass - backward_emass)/(2*dqi);
    }
    return grad_res;
}

int main(){
    double time = 0;
    const Eigen::Vector3d NORMAL_VECTOR = {0,0,1};
    const double dt = 0.002;
    Eigen::Vector3d q = configuration(0);
    Eigen::Vector3d q_dot = configuration_dot(0);

    Eigen::Vector3d old_q = q;

    double dmdt_classic = 0;
    double emass = 0;
    double old_emass = effective_mass(q, NORMAL_VECTOR);
    double dmdt_product = 0;
    double dmdt_central_diff_forward = 0;
    double dmdt_central_diff_backward = 0;
    double dmdt_central_diff = 0;
    double relative_error_classic = 0;
    double relative_error_product = 0;

    while (time <= 1) {
        std::cout << "time = " << time << std::endl;

        q = configuration(time);
        q_dot = configuration_dot(time);
        emass = effective_mass(q, NORMAL_VECTOR);
        if (time == 0)
        {
            time+=dt;
            continue;
        }
        dmdt_classic = (emass - old_emass)/dt;
        dmdt_product = emass_grad_central_differences(emass, 
                                                    time, 
                                                    old_q,
                                                    NORMAL_VECTOR)
                                                    .transpose() 
                                                    * q_dot;
        dmdt_central_diff_forward = (effective_mass(configuration(time+dt), NORMAL_VECTOR));
        dmdt_central_diff_backward = (effective_mass(configuration(time-dt), NORMAL_VECTOR));
        dmdt_central_diff = (dmdt_central_diff_forward - dmdt_central_diff_backward) / (2*dt);
        relative_error_classic = std::abs((dmdt_classic - dmdt_central_diff)/(dmdt_central_diff));
        relative_error_product = std::abs((dmdt_product - dmdt_central_diff)/(dmdt_central_diff));

        std::cout << "dmdt classic = " << dmdt_classic << std::endl;
        std::cout << "dmdt by product = " << dmdt_product << std::endl;
        std::cout << "dmdt by central diff = " << dmdt_central_diff << std::endl;
        std::cout << "relative error classic = " << 100*relative_error_classic << "%" << std::endl;
        std::cout << "relative error product = " << 100*relative_error_product << "%" << std::endl;
        
        old_q = q;
        old_emass = emass;
        
        time+=dt;
        std::cout << " -------------------- " << std::endl;
    }
    return 0;
}