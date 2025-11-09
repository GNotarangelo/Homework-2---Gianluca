// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath> // For std::sin

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  //Vision Control

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

// Functions from the kdl_control.cpp
extern std::vector<double> toStdVector(const Eigen::VectorXd& vector);
extern KDL::Vector toKDL(const Eigen::Vector3d& vector);
extern Eigen::Vector3d computeLinearError(const Eigen::Vector3d& desired, const Eigen::Vector3d& current);
extern Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& desired, const Eigen::Matrix3d& current);
extern Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& M, double r_threshold = 1e-6);

typedef Eigen::Matrix<double, 6, 1> Vector6d;



class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            double default_end_x = init_position[0];
            double default_end_y = -init_position[1];
            double default_end_z = init_position[2];

            

            //declaring parameters as ROS2 parameters
            declare_parameter("traj_duration",3.0);
            declare_parameter("acc_duration",0.5);
            declare_parameter("total_time",0.15);
            declare_parameter("trajectory_len",150);
            declare_parameter("Kp",5);
            declare_parameter("end_position.x", default_end_x);
            declare_parameter("end_position.y", default_end_y);
            declare_parameter("end_position.z", default_end_z);

            //storing them in vars
            get_parameter("traj_duration", this->traj_duration);
            get_parameter("acc_duration", this->acc_duration);
            get_parameter("total_time", this->total_time);
            get_parameter("trajectory_len", this->trajectory_len);
            get_parameter("Kp", this->Kp);
            get_parameter("end_position.x", this->ex);
            get_parameter("end_position.y", this->ey);
            get_parameter("end_position.z", this->ez);

            Eigen::Vector3d end_position(this->ex, this->ey, this->ez);

            
            //RCLCPP_INFO(this->get_logger(), "KDL parameter value: %f", traj_duration);
            
            RCLCPP_INFO(this->get_logger(), "Posizione finale traiettoria: [x: %.3f, y: %.3f, z: %.3f]",
            end_position.x(), end_position.y(), end_position.z());

            //declare_parameter("end_position",[0.0, 0.0, 0.1]);

            // declare cmd_interface parameter (position, velocity, velocity_ctrl_null, effort, vision)
            declare_parameter("cmd_interface", "position"); // default to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "velocity_ctrl_null" || cmd_interface_ == "effort" || cmd_interface_ == "vision"
            ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity', 'effort' or 'vision' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }
            
            // Vision Control
            if (cmd_interface_ == "vision") {
                // Subscriber to ArUco marker pose !!!!!CHECK NAME OF TOPIC !!!!!!
                arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, 
                    std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

                // Wait for the ArUco topic
                while(!aruco_pose_available_){
                    RCLCPP_INFO(this->get_logger(), "Waiting for ArUco marker pose data! Topic: /aruco_single/pose ...");
                    rclcpp::spin_some(node_handle_);
                }
                RCLCPP_INFO(this->get_logger(), "ArUco marker pose received. Starting vision control.");
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            controller_ = std::make_shared<KDLController>(*robot_);

            


            // Plan trajectory
            //double traj_duration = 1.5, acc_duration = 0.5, 
            double traj_radius = 0.15;

            // Retrieve the first trajectory point
            
            // Plan trajectory (Only plan if not in vision mode)
            if (cmd_interface_ != "vision") {
                double traj_radius = 0.15;

                // Retrieve the first trajectory point
                if(traj_type_ == "linear"){
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }
            }
            
            // // Retrieve the first trajectory point
            // trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            // Publisher and Timer Setup
            if(cmd_interface_ == "position"){
                // Create cmd publisher for Position Control
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            }
            else if(cmd_interface_ == "velocity" || cmd_interface_ == "velocity_ctrl_null" || cmd_interface_ == "vision"){
                // Create cmd publisher for Velocity Control (includes vision)
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher for Effort Control
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            } 
            
            // Set up the main control loop timer (used by all modes)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

            // Initial command setup (only relevant if not vision, where commands start at zero/rest)
            if (cmd_interface_ == "position") {
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            } else if (cmd_interface_ == "velocity" || cmd_interface_ == "velocity_ctrl_null" || cmd_interface_ == "vision") {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0; // Start at zero velocity
                }
            } else if (cmd_interface_ == "effort") {
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:
    
    	// Vision callback function
        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& pose_msg){
            aruco_pose_available_ = true;
            
            // Convert ROS Pose into KDL Frame (position and orientation)
            cPo_frame_.p = KDL::Vector(
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z
            );
            
            // KDL::Rotation is constructed from the quaternion
            cPo_frame_.M = KDL::Rotation::Quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w
            );
        }

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            //double total_time = 1.5; // 
            //int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            //int Kp = 5;
            t_+=dt;

            // Vision control runs indefinitely, not based on total_time
            if (cmd_interface_ == "vision" || t_ < total_time){
            
                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type

                // --- TRAJECTORY PLANNING (Skipped if in Vision Mode) ---
                if (cmd_interface_ != "vision") {
                    if(traj_type_ == "linear"){
                        if(s_type_ == "trapezoidal")
                        {
                            p_ = planner_.linear_traj_trapezoidal(t_);
                        }else if(s_type_ == "cubic")
                        {
                            p_ = planner_.linear_traj_cubic(t_);
                        }
                    } 
                    else if(traj_type_ == "circular")
                    {
                        if(s_type_ == "trapezoidal")
                        {
                            p_ = planner_.circular_traj_trapezoidal(t_);
                        }else if(s_type_ == "cubic")
                        {
                            p_ = planner_.circular_traj_cubic(t_);
                        }
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p_.vel + Kp*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                }
                else if(cmd_interface_ == "velocity_ctrl_null"){
                    // IL TUO CONTROLLER (P + NullSpace)
                    RCLCPP_INFO_ONCE(this->get_logger(), "Usando controller: P + NullSpace");

                    // Definisci la posa desiderata (posizione dal planner, orientamento fisso)
                    KDL::Frame desFrame;
                    desFrame.M = init_cart_pose_.M; // Orientamento fisso
                    desFrame.p = toKDL(p_.pos);

                    // Chiama la tua funzione
                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(desFrame, 
                                                                                (double)Kp, 
                                                                                (double)Kp);
                }
                else if(cmd_interface_ == "vision"){
                    // VISION CONTROL: Use the stored cPo_frame_ from the ArUco subscriber
                    RCLCPP_INFO_ONCE(this->get_logger(), "Usando controller: Vision Control (Look-at-Point)");
                    
                    if (aruco_pose_available_) {
                        joint_velocities_cmd_.data = controller_->vision_ctrl(cPo_frame_);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "ArUco pose not yet available for vision control loop!");
                    }
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Setting Commands for Publishing
                if(cmd_interface_ == "position"){
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity" || cmd_interface_ == "velocity_ctrl_null" || cmd_interface_ == "vision"){
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

		

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;

        KDL::Frame init_cart_pose_;

        //vars
        double ex,ey,ez;
        double traj_duration;
        double acc_duration;
        double total_time;
        int trajectory_len;
        int Kp;
        
        // Vision Control
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
        bool aruco_pose_available_ = false;
        KDL::Frame cPo_frame_; // Pose of the tag relative to camera
        
        
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
