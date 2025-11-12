#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

//Vision Control
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

Eigen::VectorXd compute_jnt_limit_avoidance(KDLRobot* robot_);

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
                           
    Eigen::VectorXd velocity_ctrl_null(KDL::Frame &_desPos,
                                         double _Kpp);
                                         
    //Vision Control
    Eigen::MatrixXd getCameraJacobian();
    Eigen::VectorXd vision_ctrl(const KDL::Frame& cPo_frame);

private:

    KDLRobot* robot_;

};

#endif
