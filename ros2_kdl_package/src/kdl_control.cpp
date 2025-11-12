#include "kdl_control.h"
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/SVD>
#include <iostream>



Eigen::VectorXd compute_jnt_limit_avoidance(KDLRobot* robot_)
{
    const double lambda = 2.0;
    
    int n = robot_->getNrJnts();
    Eigen::VectorXd q = robot_->getJntValues(); 
    Eigen::MatrixXd limits = robot_->getJntLimits(); 
    double epsilon = 1e-3; 

    Eigen::VectorXd q_dot_s0(n);
    q_dot_s0.setZero();

    for (int i = 0; i < n; ++i)
    {
        double q_i = q(i); 
        double q_min = limits(i, 0);
        double q_max = limits(i, 1);

        double range = q_max - q_min;
        double to_max = q_max - q_i;
        double to_min = q_i - q_min;

        // Check for denominator approaching zero
        double den_sqrt = to_max * to_min;
        if (std::abs(den_sqrt) < epsilon) {
            continue; // Avoid division by zero
        }

        double numerator_term = (q_i - q_min) - (q_max - q_i); 
        double numerator = (range * range) * numerator_term;
        double denominator = den_sqrt * den_sqrt; 

        
        q_dot_s0(i) = (1.0 / lambda) * (numerator / (denominator + epsilon));
    }
    return q_dot_s0;
}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd, 
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    
}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::Frame &_desPos,
                                                   double _Kpp)
{
    // Current state and jacobian
    KDL::Frame    x_curr = robot_->getEEFrame();
    KDL::Jacobian J      = robot_->getEEJacobian();
    Eigen::MatrixXd J_eigen = J.data;
    
    // computing pose error
    KDL::Twist error_pose = KDL::diff(x_curr, _desPos);
    
    // Computing command velocity
    Eigen::VectorXd x_dot_cmd_eigen(6);
    x_dot_cmd_eigen(0) = _Kpp * error_pose.vel.x();
    x_dot_cmd_eigen(1) = _Kpp * error_pose.vel.y();
    x_dot_cmd_eigen(2) = _Kpp * error_pose.vel.z();
    x_dot_cmd_eigen(3) = _Kpp * error_pose.rot.x();
    x_dot_cmd_eigen(4) = _Kpp * error_pose.rot.y();
    x_dot_cmd_eigen(5) = _Kpp * error_pose.rot.z();
    
    // computing pseudo-inverse with damping for robustness
    int n = robot_->getNrJnts();
    double damping = 0.01;  // Small damping factor
    Eigen::MatrixXd J_pinv;
    
    // Use SVD for a more robust pseudo-inverse
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eigen, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().rows());
    for (int i = 0; i < singularValues.size(); ++i) {
        S_inv(i, i) = singularValues(i) / (singularValues(i) * singularValues(i) + damping * damping);
    }
    J_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();
    
    
    // Computing joint velocities
    Eigen::VectorXd q_dot_task = J_pinv * x_dot_cmd_eigen;
    
    // optimizing second objective exploiting null space
    Eigen::VectorXd q_dot_s0 = compute_jnt_limit_avoidance(robot_);
    
    // computing null space projector
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd P = I_n - (J_pinv * J_eigen); 
    
    // putting everything together
    Eigen::VectorXd q_dot_cmd = q_dot_task + (P * q_dot_s0);
    
    // Final safety: limit overall joint velocities
    double max_joint_vel = 1.0;  // rad/s
    for (int i = 0; i < n; ++i) {
        if (std::abs(q_dot_cmd(i)) > max_joint_vel) {
            q_dot_cmd(i) = (q_dot_cmd(i) > 0) ? max_joint_vel : -max_joint_vel;
        }
    }
    
    return q_dot_cmd;
}

Eigen::MatrixXd KDLController::getCameraJacobian() {
    
    // 1. Get the EE Jacobian (J_base_EE)
    Eigen::MatrixXd J_EE = robot_->getEEJacobian().data; 
    
    // 2. Get the *current* EE pose (T_base_EE) to find its orientation
    KDL::Frame T_base_EE = robot_->getEEFrame();
    Eigen::Matrix3d R_base_EE = toEigen(T_base_EE.M);

    // 3. Get the *static* offset from EE to Camera (T_EE_cam)
    KDL::Frame T_EE_Camera = robot_->getEECameraFrameOffset(); 
    Eigen::Vector3d p_EE_cam_in_EE = toEigen(T_EE_Camera.p); // p_EE_cam (expressed in EE frame)
    
    // 4. Calculate the vector from EE to Camera in the base frame
    Eigen::Vector3d p_EE_cam_in_base = R_base_EE * p_EE_cam_in_EE;
    
    
    Eigen::MatrixXd X_EE_cam(6, 6);
    X_EE_cam.setIdentity();
    X_EE_cam.block<3, 3>(0, 3) = -skew(p_EE_cam_in_base); 

    // 6. Calculate the final Camera Jacobian
    Eigen::MatrixXd J_c = X_EE_cam * J_EE;
    
    return J_c;
}


Eigen::VectorXd KDLController::vision_ctrl(const KDL::Frame& imagePo_frame) {
    
    int n = robot_->getNrJnts();
    
    // Step 1: Extract cPo from ArUco 
    Eigen::Vector3d cPo = toEigen(imagePo_frame.p);
    double norm_cPo = cPo.norm();
    
    // Safety check
    if (norm_cPo < 0.05) { // 5 cm
        std::cerr << "[VISION] ERROR: Marker too close! (Dist: " << norm_cPo << "m)" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    
    if (cPo.z() < 0.05) {
        std::cerr << "[VISION] ERROR: Marker behind camera or too close!" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    
    
    // Step 2: Compute s 
    Eigen::Vector3d s = cPo / norm_cPo;
    Eigen::Vector3d s_desired(0.0, 0.0, 1.0);  // Marker centered along Z
    
    // error
    Eigen::Vector3d error = s_desired - s;
    double error_norm = error.norm();
    
    //marker centered
    double error_threshold = 0.02;
    if (error_norm < error_threshold) {
        return Eigen::VectorXd::Zero(n);
    }
    
 
    // Step 3: Get current camera rotation Rc 
    // Get current camera pose
    KDL::Frame T_base_EE = robot_->getEEFrame();
    KDL::Frame T_EE_Camera = robot_->getEECameraFrameOffset();
    KDL::Frame T_base_Camera = T_base_EE * T_EE_Camera;
    
    Eigen::Matrix3d Rc = toEigen(T_base_Camera.M);  // Camera rotation in base frame
    Eigen::Matrix3d Rc_T = Rc.transpose();
    
    // Build R matrix from Equation 5: R = [Rc^T  0; 0  Rc^T]
    Eigen::MatrixXd R_matrix(6, 6);
    R_matrix.setZero();
    R_matrix.block<3, 3>(0, 0) = Rc_T;
    R_matrix.block<3, 3>(3, 3) = Rc_T;
    
    
    // Step 4: Compute L(s)
    // L(s) = [-(1/||cPo||)(I - ss^T) | S(s)] * R
    Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I_minus_ssT = I_3 - s * s.transpose();
    
    Eigen::Matrix<double, 3, 6> L_inner;
    L_inner.block<3, 3>(0, 0) = -(1.0 / norm_cPo) * I_minus_ssT;
    L_inner.block<3, 3>(0, 3) = skew(s);
    
    // Apply R matrix: L(s) = L_inner * R
    Eigen::Matrix<double, 3, 6> L_s = L_inner * R_matrix;
    
    
    Eigen::MatrixXd J_c = getCameraJacobian(); 
    
    
    // Step 6: Compute L(s)Jc
    Eigen::MatrixXd L_Jc = L_s * J_c;
    
    
    // Step 7: Compute damped pseudoinverse (L(s)Jc)†
    
    double damping = 0.05;
    Eigen::MatrixXd L_Jc_T = L_Jc.transpose();
    Eigen::MatrixXd A = L_Jc * L_Jc_T + damping * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd L_Jc_pinv = L_Jc_T * A.inverse();
    
    // Step 8: Compute null-space projector N = I - (L(s)Jc)†L(s)Jc
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd N = I_n - L_Jc_pinv * L_Jc;
    
    // Step 9: Secondary task q_dot_0 (joint limit avoidance )
    Eigen::VectorXd q_dot_0 = compute_jnt_limit_avoidance(robot_);
    
    // Clamp secondary task to reasonable values (prevent dominance)
    double max_secondary = 0.1;
    q_dot_0 = q_dot_0.cwiseMax(-max_secondary).cwiseMin(max_secondary);
    
    
    // Step 10: Build gain matrix K (diagonal)
    double k_gain = 2; 
    
    // Step 11: Apply control law 
    // q_dot = K(L(s)Jc)† * (sd - s) + N * q_dot_0
    Eigen::VectorXd q_dot_primary = k_gain * L_Jc_pinv * error;
    Eigen::VectorXd q_dot_secondary = N * q_dot_0;
    Eigen::VectorXd q_dot_cmd = q_dot_primary + q_dot_secondary;
    
    // Step 12: Velocity saturation for safety
    double max_joint_vel = 1; // rad/s
    for (int i = 0; i < n; ++i) {
        if (std::abs(q_dot_cmd(i)) > max_joint_vel) {
            q_dot_cmd(i) = (q_dot_cmd(i) > 0) ? max_joint_vel : -max_joint_vel;
        }
    }
    
    
    return q_dot_cmd;
}