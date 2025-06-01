/**
 * @file drone_motion_controller.hpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Definition of the DroneMotionController class for advanced drone motion patterns in ROS2.
 * @version 0.1
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024
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
 * @brief Enum for the drone's motion states
 */
enum class MotionState {
    SETTING_ALTITUDE,
    TURNING_RIGHT_1,
    SINE_WAVE_XY,
    TURNING_RIGHT_2,
    SPIRAL_MOTION,
    TURNING_RIGHT_3,
    SINE_WAVE_Z,
    FINAL_HOVER
};

/**
 * @brief ROS2 node for advanced drone motion patterns (sine, spiral, etc.)
 */
class DroneMotionController : public rclcpp::Node {
 public:
    DroneMotionController();
    ~DroneMotionController() = default;

 private:
    // ROS2 publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State machine
    MotionState current_state_{MotionState::SETTING_ALTITUDE};
    rclcpp::Time state_start_time_;
    double elapsed_in_state_{0.0};

    // Parameters (tune as needed)
    double hover_height_{1.0};         // Target hover height (meters)
    double altitude_kp_{1.0};          // Proportional gain for altitude
    double turn_speed_{0.6};           // Angular speed for right turns (rad/s)
    double turn_duration_{1.5};        // Duration for 90° right turn (seconds)
    double sine_amplitude_{0.5};       // Amplitude for sine wave (meters)
    double sine_frequency_{0.7};       // Frequency for sine wave (Hz)
    double forward_speed_{0.3};        // Forward speed (m/s)
    double spiral_v0_{0.1};            // Initial forward speed for spiral (m/s)
    double spiral_w0_{0.3};            // Initial angular speed for spiral (rad/s)
    double spiral_k_{0.08};            // Spiral growth rate (per second)
    double spiral_duration_{7.0};      // Duration of spiral motion (seconds)
    double sine_duration_{7.0};        // Duration of sine wave motions (seconds)
    double hover_duration_{2.0};       // Duration to hover at each hover state (seconds)
    double altitude_tolerance_{0.05};  // Tolerance for reaching hover height (meters)

    // Altitude control state
    double current_altitude_{0.0};     // Simulated current altitude (for demo)
    double altitude_error_{0.0};

    // Main control loop
    void control_loop();

    // State transition helper
    void transition_to(MotionState next);

    // Command helpers for each state
    void set_altitude();
    void turn_right();
    void sine_wave_xy();
    void spiral_motion();
    void sine_wave_z();
    void final_hover();

    // Utility
    double get_time_in_state();
    void publish_velocity(double x, double y, double z, double yaw);
};

