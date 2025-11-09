#include "kdl_control.h"
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/SVD>
#include <iostream>

// These functions convert KDL types to Eigen and handle necessary matrix math.


//Converts a KDL Vector to an Eigen Vector3d.
 
Eigen::Vector3d toEigen(const KDL::Vector& kdl_vec) {
    return Eigen::Vector3d(kdl_vec.x(), kdl_vec.y(), kdl_vec.z());
}


//Converts a KDL Rotation to an Eigen Matrix3d.
 
Eigen::Matrix3d toEigen(const KDL::Rotation& kdl_rot) {
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = kdl_rot(i, j);
        }
    }
    return mat;
}


//Calculates the skew-symmetric matrix S(v) of a vector v.

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0.0, -v(2),  v(1),
          v(2),  0.0, -v(0),
         -v(1),  v(0),  0.0;
    return S;
}


//Calculates the Moore-Penrose pseudoinverse using SVD.
// mat: The input matrix.
// tolerance: The tolerance below which singular values are treated as zero.

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat, double tolerance = 1e-6) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Extract U, V, and Sigma (singular values)
    const Eigen::MatrixXd& U = svd.matrixU();
    const Eigen::MatrixXd& V = svd.matrixV();
    const Eigen::VectorXd& S = svd.singularValues();

    // Calculate reciprocal of singular values and filter by tolerance
    Eigen::VectorXd S_inv(S.size());
    for (int i = 0; i < S.size(); ++i) {
        if (S(i) > tolerance) {
            S_inv(i) = 1.0 / S(i);
        } else {
            S_inv(i) = 0.0;
        }
    }

    return V * S_inv.asDiagonal() * U.transpose();
}


//Calculates joint velocities (q_dot_0) to push joints away from limits.

Eigen::VectorXd compute_jnt_limit_avoidance(KDLRobot* robot_)
{
    const double lambda = 2.0;
    
    int n = robot_->getNrJnts();
    Eigen::VectorXd q = robot_->getJntValues(); 
    Eigen::MatrixXd limits = robot_->getJntLimits(); 
    double epsilon = 1e-9; 

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

        double numerator_term = (q_i - q_min) - (q_max - q_i);
        double numerator = (range * range) * numerator_term;
        double denominator = (to_max * to_min) * (to_max * to_min);

        // Gradient of the redundancy measure (H), scaled.
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
    Eigen::VectorXd q_dot_cmd;
    q_dot_cmd.resize(robot_->getNrJnts());
    q_dot_cmd.setZero();
    return q_dot_cmd;
}


// Calculates the Camera Jacobian Jc = R_C_EE * J_EE.

Eigen::MatrixXd KDLController::getCameraJacobian() {
    
    // 1. Get the EE Jacobian (J_EE) in the base frame
    Eigen::MatrixXd J_EE = robot_->getEEJacobian().data; 
    
    // 2. Get the fixed transformation from EE to Camera (T_EE_C)
    KDL::Frame T_EE_C_KDL = robot_->getEECameraFrameOffset(); 
    
    // 3. Extract the rotation R_EE_C (Rotation of Camera in EE frame)
    Eigen::Matrix3d R_EE_C = toEigen(T_EE_C_KDL.M); 
    
    // 4. Calculate R_C_EE = R_EE_C^T
    Eigen::Matrix3d R_C_EE = R_EE_C.transpose();

    // 5. Construct the 6x6 spatial transformation matrix (R)
    // R maps EE spatial velocity (in base frame) to Camera spatial velocity (in camera frame)
    Eigen::MatrixXd R(6, 6);
    R.setZero();
    R.block<3, 3>(0, 0) = R_C_EE; // Rotation for linear velocity
    R.block<3, 3>(3, 3) = R_C_EE; // Rotation for angular velocity

    // Jc = R * J_EE (relates joint velocity to camera spatial velocity)
    Eigen::MatrixXd J_c = R * J_EE;
    return J_c;
}


//Implements the vision-based Look-At-Point controller.
// q_dot = K * (L(s)Jc)† * error + N * q_dot_0
// cPo_frame: Pose of the marker (Object O) relative to the Camera (C).

Eigen::VectorXd KDLController::vision_ctrl(const KDL::Frame& cPo_frame) {
    
    // 1. Extract and Convert Data
    Eigen::Vector3d cPo = toEigen(cPo_frame.p);
    Eigen::Matrix3d Rc = toEigen(cPo_frame.M); 
    double norm_cPo = cPo.norm();
    int n = robot_->getNrJnts();

    // Safety check: ensure the marker is not at the camera center
    if (norm_cPo < 1e-6) {
        std::cerr << "[VISION_CTRL ERROR] Marker too close to camera center. Returning zero velocity." << std::endl;
        Eigen::VectorXd zero_cmd(n);
        zero_cmd.setZero();
        return zero_cmd;
    }
    
    // 2. Compute the feature vector 's' (Equation 4)
    Eigen::Vector3d s = cPo / norm_cPo; 
    Eigen::Vector3d sd; sd << 0.0, 0.0, 1.0; // Desired feature vector (look straight along Z-axis)
    
    // 3. Compute the L(s) matrix (Image Jacobian part) (Equation 5)
    Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
    
    // R_6 matrix (Transformation of spatial velocity into camera frame)
    Eigen::MatrixXd R_6(6, 6);
    R_6.setZero();
    R_6.block<3, 3>(0, 0) = Rc.transpose(); 
    R_6.block<3, 3>(3, 3) = Rc.transpose(); 

    // Inner term of L(s) (Maps camera velocity to feature velocity)
    Eigen::Matrix<double, 3, 6> L_inner;
    
    // Term 1: Linear velocity part (Translational)
    Eigen::Matrix3d P_s = I_3 - s * s.transpose(); // Projection matrix (I - s*s^T)
    L_inner.block<3, 3>(0, 0) = - (1.0 / norm_cPo) * P_s; 
    
    // Term 2: Angular velocity part (Rotational)
    L_inner.block<3, 3>(0, 3) = - (1.0 / norm_cPo) * P_s * skew_symmetric(s); 

    // L(s) = L_inner * R_6
    Eigen::Matrix<double, 3, 6> L_s = L_inner * R_6; 

    // 4. Compute the Full Image Jacobian L(s)Jc
    Eigen::MatrixXd Jc = getCameraJacobian(); 
    Eigen::MatrixXd ImageJacobian = L_s * Jc; 

    // 5. Compute Control Error and Gains
    Eigen::Vector3d error = sd - s; // Control error in feature space (s_d - s)
    
    // Use a fixed scalar gain Kp_scalar=5.0 for simplicity, assuming this is read from ROS Kp parameter
    double Kp_scalar = 5.0; 
    Eigen::VectorXd K_mat = Kp_scalar * Eigen::VectorXd::Ones(n);
    Eigen::MatrixXd K = K_mat.asDiagonal(); // Diagonal gain matrix K

    // 6. Compute Pseudoinverse and Null-Space Projector
    Eigen::MatrixXd pseudoInverse = pseudoinverse(ImageJacobian);
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - pseudoInverse * ImageJacobian; // Null-Space Projector N
    
    // 7. Secondary Task (q_dot_0)
    Eigen::VectorXd q_dot_s0 = compute_jnt_limit_avoidance(robot_); 
    
    // 8. Apply Control Law (Equation 3)
    // q_dot = K * (L(s)Jc)† * error + N * q_dot_0
    
    Eigen::VectorXd q_dot_primary = K * pseudoInverse * error; 
    Eigen::VectorXd q_dot_secondary = N * q_dot_s0;

    Eigen::VectorXd q_dot_cmd = q_dot_primary + q_dot_secondary;

    return q_dot_cmd;
}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::Frame &_desPos,
                                         double _Kpp, double _Kpo)
{

    // === PASSO 1: CALCOLARE LA SOLUZIONE DEL TASK PRIMARIO ===
    
    // 1a. Ottieni stato attuale e Jacobiano
    KDL::Frame    x_curr = robot_->getEEFrame();
    KDL::Jacobian J      = robot_->getEEJacobian();
    Eigen::MatrixXd J_eigen = J.data;
    int n = robot_->getNrJnts();

    // 1b. Calcola errore di posa
    KDL::Twist error_pose = KDL::diff(x_curr, _desPos);

    // 1c. Calcola velocità di comando (proporzionale)
    KDL::Twist x_dot_cmd_kdl;
    x_dot_cmd_kdl.vel = _Kpp * error_pose.vel;
    x_dot_cmd_kdl.rot = _Kpo * error_pose.rot;

    Eigen::VectorXd x_dot_cmd_eigen(6);
    x_dot_cmd_eigen(0) = x_dot_cmd_kdl.vel.x();
    x_dot_cmd_eigen(1) = x_dot_cmd_kdl.vel.y();
    x_dot_cmd_eigen(2) = x_dot_cmd_kdl.vel.z();
    x_dot_cmd_eigen(3) = x_dot_cmd_kdl.rot.x();
    x_dot_cmd_eigen(4) = x_dot_cmd_kdl.rot.y();
    x_dot_cmd_eigen(5) = x_dot_cmd_kdl.rot.z();

    // 1d. Calcola pseudo-inversa J_pinv (J_dagger) con SVD
    Eigen::MatrixXd J_pinv = pseudoinverse(J_eigen);

    // 1e. Calcola velocità giunti per il task primario
    Eigen::VectorXd q_dot_task = J_pinv * x_dot_cmd_eigen;


    // === PASSO 2: CALCOLARE VELOCITÀ SPAZIO NULLO (Eq. 2) ===
    Eigen::VectorXd q_dot_s0 = compute_jnt_limit_avoidance(robot_);


    // === PASSO 3: CALCOLARE PROIETTORE SPAZIO NULLO ===
    
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd P = I_n - (J_pinv * J_eigen); // Proiettore (I - J_dagger * J)


    // === PASSO 4: COMBINARE TUTTO (Eq. 1) ===

    Eigen::VectorXd q_dot_cmd = q_dot_task + (P * q_dot_s0);

    return q_dot_cmd;
}

