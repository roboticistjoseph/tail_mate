/**
 * @file drone_motion_controller.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Implementation of the DroneMotionController class for advanced drone motion patterns in ROS2.
 * @version 0.1
 * @date 2024-06-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// including necessary libraries and headers
#include "../include/drone_motion_controller.hpp"
#include <cmath>

using namespace std::chrono_literals;  // NOLINT

DroneMotionController::DroneMotionController()
: Node("drone_motion_controller") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneMotionController::control_loop, this));
    state_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Drone Motion Controller started.");
}

void DroneMotionController::control_loop() {
    elapsed_in_state_ = get_time_in_state();

    switch (current_state_) {
        case MotionState::SETTING_ALTITUDE:
            set_altitude();
            break;
        case MotionState::TURNING_RIGHT_1:
        case MotionState::TURNING_RIGHT_2:
        case MotionState::TURNING_RIGHT_3:
            turn_right();
            break;
        case MotionState::SINE_WAVE_XY:
            sine_wave_xy();
            break;
        case MotionState::SPIRAL_MOTION:
            spiral_motion();
            break;
        case MotionState::SINE_WAVE_Z:
            sine_wave_z();
            break;
        case MotionState::FINAL_HOVER:
            final_hover();
            break;
    }
}

void DroneMotionController::set_altitude() {
    // Simulate altitude error (replace with real sensor in real drone)
    altitude_error_ = hover_height_ - current_altitude_;
    double vz = altitude_kp_ * altitude_error_;

    // Clamp vertical speed for safety
    if (vz > 0.5) vz = 0.5;
    if (vz < -0.5) vz = -0.5;

    // Simulate altitude update (for demo; having issues with imu sensor feedback)
    current_altitude_ += vz * 0.1;  // dt = 0.1s

    // Publish only vertical velocity
    publish_velocity(0.0, 0.0, vz, 0.0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[ALTITUDE] Error: %.2f, Current: %.2f, vz: %.2f", altitude_error_, current_altitude_, vz);

    // Transition when within tolerance or after timeout
    if (std::abs(altitude_error_) < altitude_tolerance_ || elapsed_in_state_ > 3.0) {
        transition_to(MotionState::TURNING_RIGHT_1);
    }
}

void DroneMotionController::turn_right() {
    publish_velocity(0.0, 0.0, 0.0, -turn_speed_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[TURN] Turning right. State: %d, Time: %.2f", static_cast<int>(current_state_), elapsed_in_state_);

    if (elapsed_in_state_ > turn_duration_) {
        switch (current_state_) {
            case MotionState::TURNING_RIGHT_1:
                transition_to(MotionState::SINE_WAVE_XY);
                break;
            case MotionState::TURNING_RIGHT_2:
                transition_to(MotionState::SPIRAL_MOTION);
                break;
            case MotionState::TURNING_RIGHT_3:
                transition_to(MotionState::SINE_WAVE_Z);
                break;
            default:
                break;
        }
    }
}

void DroneMotionController::sine_wave_xy() {
    double t = elapsed_in_state_;
    double y = sine_amplitude_ * std::sin(2 * M_PI * sine_frequency_ * t);
    publish_velocity(forward_speed_, y, 0.0, 0.0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[SINE_XY] t: %.2f, x: %.2f, y: %.2f", t, forward_speed_, y);

    if (t > sine_duration_) {
        transition_to(MotionState::TURNING_RIGHT_2);
    }
}

void DroneMotionController::spiral_motion() {
    double t = elapsed_in_state_;
    double vx = spiral_v0_ + spiral_k_ * t;
    double wz = spiral_w0_ + spiral_k_ * t;
    publish_velocity(vx, 0.0, 0.0, wz);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[SPIRAL] t: %.2f, vx: %.2f, wz: %.2f", t, vx, wz);

    if (t > spiral_duration_) {
        transition_to(MotionState::TURNING_RIGHT_3);
    }
}

void DroneMotionController::sine_wave_z() {
    double t = elapsed_in_state_;
    double z = sine_amplitude_ * std::sin(2 * M_PI * sine_frequency_ * t);
    publish_velocity(forward_speed_, 0.0, z, 0.0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[SINE_Z] t: %.2f, x: %.2f, z: %.2f", t, forward_speed_, z);

    if (t > sine_duration_) {
        transition_to(MotionState::FINAL_HOVER);
    }
}

void DroneMotionController::final_hover() {
    publish_velocity(0.0, 0.0, 0.0, 0.0);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[FINAL_HOVER] Holding position at height %.2f", hover_height_);
    // Remain in this state indefinitely
}

void DroneMotionController::transition_to(MotionState next) {
    current_state_ = next;
    state_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Transitioning to state: %d", static_cast<int>(next));
}

double DroneMotionController::get_time_in_state() {
    return (this->now() - state_start_time_).seconds();
}

void DroneMotionController::publish_velocity(double x, double y, double z, double yaw) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    msg.linear.z = z;
    msg.angular.z = yaw;
    cmd_vel_pub_->publish(msg);
}

/**
 * @brief Main function to initialize the ROS2 node and start the DroneMotionController.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneMotionController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
