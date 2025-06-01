/**
 * @file waypoint_navigation.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief C++ implementation of a Turtlebot3 patrol route using ROS2 Nav2 action client.
 * @version 0.1
 * @date 2025-06-01
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// including necessary libraries and headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT

class SecurityPatrol : public rclcpp::Node {
 public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    

    SecurityPatrol()
    : Node("security_patrol")
    {
        // Create action client for NavigateThroughPoses
        client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

        // Create publisher for initial pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Set initial pose and create route
        setInitialPose();
        createRoute();
        // Wait for Nav2 action server to be ready
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        if (!client_ptr_->wait_for_action_server(20s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
            rclcpp::shutdown();
            return;
        }
        // Start patrol (one lap only)
        patrol();
    }

 private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

    // Create publisher for initial pose
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    void setInitialPose() {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = this->get_clock()->now();
        initial_pose.pose.pose.position.x = 0.0;
        initial_pose.pose.pose.position.y = 0.0;
        initial_pose.pose.pose.position.z = 0.0;
        initial_pose.pose.pose.orientation.x = 0.0;
        initial_pose.pose.pose.orientation.y = 0.0;
        initial_pose.pose.pose.orientation.z = 0.0;
        initial_pose.pose.pose.orientation.w = 0.1;
        // Set a reasonable covariance (identity matrix for demo)
        for (int i = 0; i < 36; ++i) initial_pose.pose.covariance[i] = 0.0;
        initial_pose.pose.covariance[0] = 0.25;   // x
        initial_pose.pose.covariance[7] = 0.25;   // y
        initial_pose.pose.covariance[35] = 0.0685; // yaw

        initial_pose_pub_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Initial pose set and published.");
    }

    void createRoute() {
        // Define your security route waypoints (same as your Python version)
        std::vector<std::pair<double, double>> security_route = {
            { 4.66,  8.99},
            {-3.56,  5.15},
            {-4.21, -3.02},
            {-7.07, -7.90},
            { 5.92, -7.95},
            { 0.08, -6.04},
            { 0.00,  0.00}
        };
        waypoints_.clear();
        for (const auto& coord : security_route) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = now();
            pose.pose.position.x = coord.first;
            pose.pose.position.y = coord.second;
            pose.pose.orientation.w = 1.0;  // Facing forward
            waypoints_.push_back(pose);
        }
        RCLCPP_INFO(this->get_logger(), "Route with %zu waypoints created.", waypoints_.size());
    }

    void patrol() {
        // Prepare goal message
        NavigateThroughPoses::Goal goal_msg;
        goal_msg.poses = waypoints_;
        goal_msg.behavior_tree = "";  // Use default behavior tree

        // Set up goal options
        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SecurityPatrol::handleResult, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&SecurityPatrol::handleFeedback, this, std::placeholders::_1, std::placeholders::_2);

        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending waypoints to Nav2...");
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void handleFeedback(
        GoalHandleNavigateThroughPoses::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        if (!feedback->current_pose.header.frame_id.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Currently at waypoint %ld/%zu",
                waypoints_.size() - feedback->number_of_poses_remaining + 1, waypoints_.size());
        }

    }

    void handleResult(const GoalHandleNavigateThroughPoses::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Security route completed successfully! Shutting down.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation canceled.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SecurityPatrol>();
    rclcpp::spin(node);
    return 0;
}
