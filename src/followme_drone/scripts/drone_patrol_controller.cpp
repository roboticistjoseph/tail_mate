/**
 * @file drone_patrol_controller.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Implementation of the DroneController class for controlling drone movement patterns in ROS2.
 * @version 0.1
 * @date 2025-05-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// including header files
#include "../include/drone_patrol_controller.hpp"

using namespace std::chrono_literals;

DronePatrolController::DronePatrolController() : Node("drone_patrol_controller") {
    // Initialize publisher
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/simple_drone/cmd_vel", 10);

    // Create timer for patrol movements
    patrol_timer_ = this->create_wall_timer(
        100ms, std::bind(&DronePatrolController::patrol_movements, this));

    RCLCPP_INFO(this->get_logger(), "Drone Patrol Controller initialized");
}

void DronePatrolController::patrol_movements() {
    static auto last_state_change = this->now();
    static int movement_sequence = 0;
    static const std::vector<PatrolState> sequence = {
        PatrolState::HOVER,
        PatrolState::FORWARD,
        PatrolState::HOVER,
        PatrolState::RIGHT,
        PatrolState::HOVER,
        PatrolState::BACKWARD,
        PatrolState::HOVER,
        PatrolState::LEFT,
        PatrolState::HOVER
    };

    // Calculate time since last state change
    auto current_time = this->now();
    double time_elapsed = (current_time - last_state_change).seconds();

    // Check if it's time to transition to next state
    if (time_elapsed >= wait_time_) {
        movement_sequence = (movement_sequence + 1) % sequence.size();
        last_state_change = current_time;
        transition_to_state(sequence[movement_sequence]);
    }

    // Execute current state behavior
    switch (current_state_) {
        case PatrolState::HOVER:
            publish_velocity(0.0, 0.0, hover_height_ - current_error_, 0.0);
            break;
        case PatrolState::FORWARD:
            publish_velocity(movement_speed_, 0.0, hover_height_ - current_error_, 0.0);
            break;
        case PatrolState::BACKWARD:
            publish_velocity(-movement_speed_, 0.0, hover_height_ - current_error_, 0.0);
            break;
        case PatrolState::LEFT:
            publish_velocity(0.0, movement_speed_, hover_height_ - current_error_, 0.0);
            break;
        case PatrolState::RIGHT:
            publish_velocity(0.0, -movement_speed_, hover_height_ - current_error_, 0.0);
            break;
        case PatrolState::TRANSITION:
            // Handle smooth transition
            break;
    }
}

double DronePatrolController::calculate_pid(double error, double dt, bool is_linear) {
    // Update integral and derivative terms
    error_integral_ += error * dt;
    double error_derivative = (error - previous_error_) / dt;
    previous_error_ = error;

    // Select appropriate gains
    double kp = is_linear ? kp_linear_ : kp_angular_;
    double ki = is_linear ? ki_linear_ : ki_angular_;
    double kd = is_linear ? kd_linear_ : kd_angular_;

    // Calculate PID output
    return kp * error + ki * error_integral_ + kd * error_derivative;
}

void DronePatrolController::publish_velocity(double linear_x, double linear_y, double linear_z, double angular_z) {
    auto twist_msg = geometry_msgs::msg::Twist();

    // Apply PID control for smooth movement
    double dt = 0.1;  // Assuming 10Hz control loop

    // Calculate PID corrections
    double x_correction = calculate_pid(linear_x, dt, true);
    double y_correction = calculate_pid(linear_y, dt, true);
    double z_correction = calculate_pid(linear_z, dt, true);
    double yaw_correction = calculate_pid(angular_z, dt, false);

    // Apply corrections to velocity commands
    twist_msg.linear.x = x_correction;
    twist_msg.linear.y = y_correction;
    twist_msg.linear.z = z_correction;
    twist_msg.angular.z = yaw_correction;

    cmd_vel_publisher_->publish(twist_msg);
}

void DronePatrolController::transition_to_state(PatrolState target_state) {
    RCLCPP_INFO(this->get_logger(), "Transitioning to new state");
    current_state_ = target_state;

    // Reset PID control variables for smooth transition
    error_integral_ = 0.0;
    previous_error_ = 0.0;
    current_error_ = 0.0;
}

// Main entry point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DronePatrolController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
