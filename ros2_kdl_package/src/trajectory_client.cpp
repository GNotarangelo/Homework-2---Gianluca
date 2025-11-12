#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/trajectory_execution.hpp"

using namespace std::chrono_literals;

class TrajectoryActionClient : public rclcpp::Node
{
public:
    using TrajectoryExecution = ros2_kdl_package::action::TrajectoryExecution;
    using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<TrajectoryExecution>;

    TrajectoryActionClient() : Node("trajectory_action_client")//constructor
    {
        this->client_ = rclcpp_action::create_client<TrajectoryExecution>(
            this,
            "execute_trajectory");
    }

    void send_goal(double traj_duration, double acc_duration,
                   std::vector<double> end_position,
                   std::string traj_type, std::string s_type)
    {
        using namespace std::placeholders;

        // Wait for action server
        if (!this->client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Create goal message
        auto goal_msg = TrajectoryExecution::Goal();
        goal_msg.traj_duration = traj_duration;
        goal_msg.acc_duration = acc_duration;
        goal_msg.end_position = end_position;
        goal_msg.traj_type = traj_type;
        goal_msg.s_type = s_type;

        RCLCPP_INFO(this->get_logger(), "Sending goal:");
        RCLCPP_INFO(this->get_logger(), "  Duration: %.2f s", traj_duration);
        RCLCPP_INFO(this->get_logger(), "  End position: [%.3f, %.3f, %.3f]",
                    end_position[0], end_position[1], end_position[2]);
        RCLCPP_INFO(this->get_logger(), "  Trajectory type: %s", traj_type.c_str());
        RCLCPP_INFO(this->get_logger(), "  S type: %s", s_type.c_str());

        // Set up goal options with callbacks
        auto send_goal_options = rclcpp_action::Client<TrajectoryExecution>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
        
        send_goal_options.feedback_callback =
            std::bind(&TrajectoryActionClient::feedback_callback, this, _1, _2);
        
        send_goal_options.result_callback =
            std::bind(&TrajectoryActionClient::result_callback, this, _1);

        // Send the goal
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<TrajectoryExecution>::SharedPtr client_;

    void goal_response_callback(const GoalHandleTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleTrajectory::SharedPtr,
        const std::shared_ptr<const TrajectoryExecution::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Progress: %.1f%% | Error norm: %.4f | Elapsed: %.2fs",
                    feedback->progress,
                    feedback->error_norm,
                    feedback->elapsed_time);
    }

    void result_callback(const GoalHandleTrajectory::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                RCLCPP_INFO(this->get_logger(), "  Final error norm: %.4f",
                            result.result->final_error_norm);
                RCLCPP_INFO(this->get_logger(), "  Execution time: %.2f s",
                            result.result->execution_time);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto action_client = std::make_shared<TrajectoryActionClient>();

    // Example: Send a goal for a linear trajectory
    std::vector<double> end_pos = {0.7, -0.5, 0.8};  // [x, y, z]
    action_client->send_goal(
        3.0,          // traj_duration
        0.5,          // acc_duration
        end_pos,      // end_position
        "linear",     // traj_type
        "trapezoidal" // s_type
    );

    rclcpp::spin(action_client);
    rclcpp::shutdown();
    
    return 0;
}