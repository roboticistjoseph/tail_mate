#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>  // Add this at the top of your file
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_util/simple_action_server.hpp"

using namespace std::chrono_literals;

class SecurityPatrol : public rclcpp::Node {
public:
    SecurityPatrol() : Node("security_patrol") {
        this->declare_parameter("control_frequency", 1.0);
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            this,
            "navigate_through_poses");
            
        initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);
            
        timer_ = create_wall_timer(
            500ms, std::bind(&SecurityPatrol::runPatrol, this));
    }

    void setInitialPose() {
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = now();
        initial_pose.pose.pose.position.x = 0.036;
        initial_pose.pose.pose.position.y = 0.009;
        initial_pose.pose.pose.orientation.z = -0.010;
        initial_pose.pose.pose.orientation.w = 0.99;
        initial_pose_pub_->publish(initial_pose);
    }

    void runPatrol() {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> route = createRoute();
        auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal_msg.poses = route;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result) {
                handleResult(result);
            };
            
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> createRoute() {
        std::vector<std::array<double, 2>> security_route = {
            {4.66, 8.99},
            {-3.56, 5.15},
            {-4.21, -3.02},
            {-7.07, -7.90},
            {5.92, -7.95},
            {0.08, -6.04},
            {0.00, 0.00}
        };

        std::vector<geometry_msgs::msg::PoseStamped> poses;
        for (auto &pt : security_route) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = now();
            pose.pose.position.x = pt[0];
            pose.pose.position.y = pt[1];
            pose.pose.orientation.w = 1.0;
            poses.push_back(pose);
        }
        return poses;
    }
    
    void handleResult(const auto &result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Route complete! Restarting...");
                // security_route_.reverse();
                std::reverse(security_route_.begin(), security_route_.end());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Route canceled");
                rclcpp::shutdown();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Route failed");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                RCLCPP_WARN(get_logger(), "Route result unknown!");
                break;
            default:
                RCLCPP_WARN(get_logger(), "Unhandled result code!");
                break;
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::array<double, 2>> security_route_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SecurityPatrol>();
    node->setInitialPose();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
