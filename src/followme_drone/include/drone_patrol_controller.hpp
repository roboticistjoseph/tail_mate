/**
 * @file drone_patrol_controller.hpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Definition of the DroneController class for controlling drone movement patterns in ROS2.
 * @version 0.1
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// defining pragma once to avoid multiple inclusions of this header file
#pragma once

// including necessary libraries and headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>  // NOLINT
#include <memory>  // NOLINT

/**
 * @brief Enumeration for different patrol movement states
 */
enum class PatrolState {
    HOVER,      // Maintain position at specified height
    FORWARD,    // Move forward
    BACKWARD,   // Move backward
    LEFT,       // Move left
    RIGHT,      // Move right
    TRANSITION  // Transitioning between states
};

/**
 * @brief Controller node for autonomous drone patrol movements
 * 
 * This node implements a patrol pattern for a drone using PID control
 * for smooth movements and transitions between different positions.
 */
class DronePatrolController : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Drone Patrol Controller object
     */
    DronePatrolController();

    /**
     * @brief Destroy the Drone Patrol Controller object
     */
    ~DronePatrolController() = default;

 private:
    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr patrol_timer_;

    // PID Control Parameters
    double kp_linear_{0.5};    // Proportional gain for linear motion
    double ki_linear_{0.1};    // Integral gain for linear motion
    double kd_linear_{0.05};   // Derivative gain for linear motion

    double kp_angular_{0.5};   // Proportional gain for angular motion
    double ki_angular_{0.1};   // Integral gain for angular motion
    double kd_angular_{0.05};  // Derivative gain for angular motion

    // Movement Parameters
    double hover_height_{1.0};     // Default hover height in meters
    double movement_speed_{0.2};   // Default movement speed in m/s
    double wait_time_{5.0};        // Wait time between movements in seconds

    // State Variables
    PatrolState current_state_{PatrolState::HOVER};
    double current_error_{0.0};
    double previous_error_{0.0};
    double error_integral_{0.0};

    // Member Functions
    /**
     * @brief Main patrol movement control function
     */
    void patrol_movements();

    /**
     * @brief Calculate PID control output
     * @param error Current error value
     * @param dt Time step
     * @param is_linear Whether calculating for linear or angular motion
     * @return double Control output
     */
    double calculate_pid(double error, double dt, bool is_linear);

    /**
     * @brief Publish velocity commands to the drone
     * @param linear_x Linear velocity in x direction
     * @param linear_y Linear velocity in y direction
     * @param linear_z Linear velocity in z direction
     * @param angular_z Angular velocity around z axis
     */
    void publish_velocity(double linear_x, double linear_y, double linear_z, double angular_z);

    /**
     * @brief Transition smoothly to target state
     * @param target_state Desired patrol state
     */
    void transition_to_state(PatrolState target_state);
};
