#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <fstream> 
#include <filesystem>
#include <thread>

#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_kdl_package/action/trajectory_execution.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "kdl/frames.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"


extern std::vector<double> toStdVector(const Eigen::VectorXd& vector);
extern KDL::Vector toKDL(const Eigen::Vector3d& vector);
extern Eigen::Vector3d computeLinearError(const Eigen::Vector3d& desired, const Eigen::Vector3d& current);
extern Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& desired, const Eigen::Matrix3d& current);
extern Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& M, double r_threshold = 1e-6);

typedef Eigen::Matrix<double, 6, 1> Vector6d;
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

using TrajectoryExecution = ros2_kdl_package::action::TrajectoryExecution;
using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<TrajectoryExecution>;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            
            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_available_ = false;

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
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;
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

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

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

            
            // Calculate dt ONCE from parameters and store as class member
            double loop_rate = trajectory_len / total_time;
            dt_ = 1.0 / loop_rate;
            loop_period_ms_ = (long)(dt_ * 1000.0);
            RCLCPP_INFO(this->get_logger(), "Controller dt set to %.4f s (%ld ms) based on parameters.", dt_, loop_period_ms_);
            

            Eigen::Vector3d end_position(this->ex, this->ey, this->ez);
            
            RCLCPP_INFO(this->get_logger(), "Posizione finale traiettoria: [x: %.3f, y: %.3f, z: %.3f]",
            end_position.x(), end_position.y(), end_position.z());

            // declare cmd_interface parameter (position, velocity,velocity_ctrl_null, vision, action)
            declare_parameter("cmd_interface", "position");
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            // Log folder path as parameter
            this->declare_parameter<std::string>("log_folder_path", "");
            this->get_parameter("log_folder_path", log_folder_path_);

            // Creating log folder if it doesn't exists
            try {
                std::filesystem::create_directories(log_folder_path_);
            } catch (const std::filesystem::filesystem_error& e) {
                RCLCPP_ERROR(get_logger(), "Impossibile creare la cartella di log: %s", e.what());
            }

            // Creating log file names
            std::string pos_filename = log_folder_path_ + "/" + cmd_interface_ + "_positions.csv";
            std::string vel_filename = log_folder_path_ + "/" + cmd_interface_ + "_velocities.csv";
            
            // Opening files
            pos_file_stream_.open(pos_filename, std::ios::out | std::ios::trunc);
            vel_file_stream_.open(vel_filename, std::ios::out | std::ios::trunc);

            // Headings
            pos_file_stream_ << "time,q1,q2,q3,q4,q5,q6,q7\n";
            vel_file_stream_ << "time,q1_dot,q2_dot,q3_dot,q4_dot,q5_dot,q6_dot,q7_dot\n";

            RCLCPP_INFO(get_logger(), "Logging posizioni in: %s", pos_filename.c_str());
            RCLCPP_INFO(get_logger(), "Logging velocità in: %s", vel_filename.c_str());
            
            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "velocity_ctrl_null" || cmd_interface_ == "effort"  || cmd_interface_ == "vision" || cmd_interface_ == "action"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity', 'velocity_ctrl_null', 'effort' or 'vision' instead..."); 
                return;
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

            // Initialize controller
            controller_ = std::make_shared<KDLController>(*robot_);

            // Initialize trajectory planner for non-vision modes
            if (cmd_interface_ != "vision") {
                // Plan trajectory
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

            

            // Vision Control Setup
            if (cmd_interface_ == "vision") {
                // Initialize the TF2 buffer and listener
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                // Subscriber to ArUco marker pose
                arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, 
                    std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));

                RCLCPP_INFO(this->get_logger(), "Waiting for ArUco marker pose data on topic: /aruco_single/pose ...");
                
                // Give some time for the first message to arrive (non-blocking approach)
                auto start_time = this->now();
                while(!aruco_pose_available_ && (this->now() - start_time).seconds() < 5.0){
                    rclcpp::spin_some(node_handle_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                }
                
                if(aruco_pose_available_) {
                    RCLCPP_INFO(this->get_logger(), "ArUco marker pose received. Starting vision control.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "No ArUco marker detected yet. Will start control when marker is visible.");
                }

                // Create velocity command publisher for vision control
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_period_ms_), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                // Initialize velocity commands to zero
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
            }
            else if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_period_ms_),
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_period_ms_), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            
            else if(cmd_interface_ == "velocity_ctrl_null"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_period_ms_), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_period_ms_), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }
            else if (cmd_interface_=="action") {
                //INITIALIZE ACTION SERVER
                executing_action_ = false;
                
                
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);

                action_server_ = rclcpp_action::create_server<TrajectoryExecution>(
                    this,
                    "execute_trajectory",
                    std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                    std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
                );
                
                RCLCPP_INFO(this->get_logger(), "Action server 'execute_trajectory' ready!");
                //END ACTION SERVER INIT
            }
            
            
            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

        ~Iiwa_pub_sub()
        {
            if (pos_file_stream_.is_open()) {
                pos_file_stream_.close();
            }
            if (vel_file_stream_.is_open()) {
                vel_file_stream_.close();
            }
        }

    private:

        // Vision callback function
        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg){
            
            cPo_frame_.p = KDL::Vector(
                pose_msg->pose.position.x,
                pose_msg->pose.position.y,
                pose_msg->pose.position.z
            );
            cPo_frame_.M = KDL::Rotation::Quaternion(
                pose_msg->pose.orientation.x,
                pose_msg->pose.orientation.y,
                pose_msg->pose.orientation.z,
                pose_msg->pose.orientation.w
            );

            aruco_pose_available_ = true;
            
            RCLCPP_INFO_ONCE(this->get_logger(), "ArUco marker detected in '%s' at [%.3f, %.3f, %.3f]",
                pose_msg->header.frame_id.c_str(),
                cPo_frame_.p.x(),
                cPo_frame_.p.y(),
                cPo_frame_.p.z());
        }

        void cmd_publisher(){
            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            
            iteration_ = iteration_ + 1;

            t_+=dt_;
            


            
            if (cmd_interface_ == "vision" || t_ < traj_duration){
                if (cmd_interface_ != "vision") {
                    // Retrieve the trajectory point based on the trajectory type
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

                if(cmd_interface_ == "vision"){
                    // VISION CONTROL: Use the stored cPo_frame_ from the ArUco subscriber
                    RCLCPP_INFO_ONCE(this->get_logger(), "Using vision control (Look-at-Point)");
                    
                    if (aruco_pose_available_) {
                        // Call vision controller with the ArUco pose
                        joint_velocities_cmd_.data = controller_->vision_ctrl(cPo_frame_);
                        
                        // Set joint velocity commands
                        for (long int i = 0; i < joint_velocities_cmd_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_cmd_(i);
                        }
                    } else {
                        RCLCPP_INFO_ONCE(this->get_logger(), 
                            "ArUco marker not visible - sending zero velocities");
                        // Send zero velocities if no marker is detected
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = 0.0;
                        }
                    }
                } else {
                    // Non-vision control modes
                    // Compute desired Frame //same orient diff pos
                    KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

                    // compute errors
                    Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                    Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

                    if(cmd_interface_ == "position"){
                        // Next Frame
                        
                        KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt_; 

                        // Compute IK
                        joint_positions_cmd_ = joint_positions_;
                        robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                        
                        // Set joint position commands
                        for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                            desired_commands_[i] = joint_positions_cmd_(i);
                        }
                    }
                    else if(cmd_interface_ == "velocity"){
                        
                        Vector6d cartvel; cartvel << p_.vel + Kp*error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        
                        // Set joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_cmd_(i);
                        }
                    }
                    else if(cmd_interface_ == "velocity_ctrl_null"){
                        // new defined controller
                        RCLCPP_INFO_ONCE(this->get_logger(), "Using nullspace controller");

                        // calling the defined function
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(desFrame, (double)Kp);
                        
                        // Set joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_cmd_(i);
                        }
                    }
                    else if(cmd_interface_ == "effort"){
                        joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                        
                        // Set joint effort commands
                        for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                            desired_commands_[i] = joint_efforts_cmd_(i);
                        }
                    }
                }
                
                //Write current positions
                pos_file_stream_ << t_;
                for (int i = 0; i < (int)robot_->getNrJnts(); ++i) {
                    pos_file_stream_ << "," << joint_positions_(i);
                }
                pos_file_stream_ << "\n" << std::flush;

                // Write computed velocities
                vel_file_stream_ << t_;
                for (int i = 0; i < (int)robot_->getNrJnts(); ++i) {
                    vel_file_stream_ << "," << joint_velocities_cmd_(i);
                }
                vel_file_stream_ << "\n"<<std::flush;

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send ZERO velocity commands to stop and maintain position
                std::vector<double> zero_commands(joint_velocities_.data.size(), 0.0);
                
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = zero_commands;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState::SharedPtr sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg->position.size(); i++){
                joint_positions_.data[i] = sensor_msg->position[i];
                joint_velocities_.data[i] = sensor_msg->velocity[i];
            }
        }

        //ACTION SERVER CALLBACK IMPLEMENTATIONS
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const TrajectoryExecution::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleTrajectory> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
        {
            std::thread{std::bind(&Iiwa_pub_sub::execute_trajectory, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute_trajectory(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing trajectory action...");
            
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TrajectoryExecution::Feedback>();
            auto result = std::make_shared<TrajectoryExecution::Result>();
            
            executing_action_ = true;
            current_goal_handle_ = goal_handle;
            
            // Update trajectory parameters from goal
            double action_traj_duration = goal->traj_duration;
            double action_acc_duration = goal->acc_duration;
            
            Eigen::Vector3d end_position(goal->end_position[0], goal->end_position[1], goal->end_position[2]);
            
            // Re-initialize planner with new goal
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame current_pose = robot_->getEEFrame();
            Eigen::Vector3d init_position(Eigen::Vector3d(current_pose.p.data));
            
            KDLPlanner action_planner;
            trajectory_point action_p;
            
            if(goal->traj_type == "linear"){
                action_planner = KDLPlanner(action_traj_duration, action_acc_duration, init_position, end_position);
            } else if(goal->traj_type == "circular"){
                double traj_radius = 0.15;
                action_planner = KDLPlanner(action_traj_duration, init_position, traj_radius, action_acc_duration);
            }
            
            // Reset time
            double action_t = 0.0;
            
            
            // Execute trajectory
            while (action_t < action_traj_duration && rclcpp::ok()) {
                // Check if goal was cancelled
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->final_error_norm = 0.0;
                    goal_handle->canceled(result);
                    executing_action_ = false;
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                
                // Update robot state
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
                
                // Get trajectory point
                if(goal->traj_type == "linear"){
                    if(goal->s_type == "trapezoidal") {
                        action_p = action_planner.linear_traj_trapezoidal(action_t);
                    } else if(goal->s_type == "cubic") {
                        action_p = action_planner.linear_traj_cubic(action_t);
                    }
                } else if(goal->traj_type == "circular") {
                    if(goal->s_type == "trapezoidal") {
                        action_p = action_planner.circular_traj_trapezoidal(action_t);
                    } else if(goal->s_type == "cubic") {
                        action_p = action_planner.circular_traj_cubic(action_t);
                    }
                }
                
                // Compute EE frame and error
                KDL::Frame cartpos = robot_->getEEFrame();
                Eigen::Vector3d error = computeLinearError(action_p.pos, Eigen::Vector3d(cartpos.p.data));
                
                // Publish feedback
                feedback->position_error = {error[0], error[1], error[2]};
                feedback->error_norm = error.norm();
                feedback->elapsed_time = action_t;
                feedback->progress = (action_t / action_traj_duration) * 100.0;
                goal_handle->publish_feedback(feedback);
                
                // Compute control
                KDL::Frame desFrame; 
                desFrame.M = cartpos.M; 
                desFrame.p = toKDL(action_p.pos);
                //velocity interface -like control but with no orientation
                //Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                Eigen::Vector3d zero_orientation_vel = {0.0, 0.0, 0.0};
                Vector6d cartvel; cartvel << action_p.vel + Kp*error, zero_orientation_vel;
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                
                // Publish commands
                for (long int i = 0; i < joint_velocities_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
                
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                
                
                action_t += dt_;
                std::this_thread::sleep_for(std::chrono::milliseconds(loop_period_ms_));
            }
            
            //stop the movement after traj
            RCLCPP_INFO(this->get_logger(), "Trajectory finished. Sending zero velocities.");
            std::vector<double> zero_commands(joint_velocities_.data.size(), 0.0);
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = zero_commands;
            cmdPublisher_->publish(cmd_msg);
            
            // Trajectory completed
            KDL::Frame final_cartpos = robot_->getEEFrame();
            Eigen::Vector3d final_error = computeLinearError(action_p.pos, Eigen::Vector3d(final_cartpos.p.data));
            
            result->success = true;
            result->final_error_norm = final_error.norm();
            result->execution_time = action_t;
            
            goal_handle->succeed(result);
            executing_action_ = false;
            
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed! Final error: %.4f", final_error.norm());
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

        std::ofstream pos_file_stream_;
        std::ofstream vel_file_stream_;
        std::string log_folder_path_;

        rclcpp_action::Server<TrajectoryExecution>::SharedPtr action_server_;
        std::shared_ptr<GoalHandleTrajectory> current_goal_handle_;
        bool executing_action_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
        bool aruco_pose_available_;
        KDL::Frame cPo_frame_;

        // TF2 member variables
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        //vars
        double ex,ey,ez;
        double traj_duration;
        double acc_duration;
        double total_time;
        int trajectory_len;
        int Kp;

        double dt_;
        long loop_period_ms_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}