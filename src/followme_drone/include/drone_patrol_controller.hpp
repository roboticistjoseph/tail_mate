/**
 * @file drone_patrol_controller.hpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Definition of the DroneController class for controlling drone movement patterns in ROS2.
 * @version 0.1
 * @date 2025-05-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// defining pragma once to avoid multiple inclusions of this header file
#pragma once

// including necessary libraries and headers
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @class DroneController
 * @brief ROS2 node for controlling a drone's movement pattern.
 *
 * This class publishes velocity commands to make the drone move in a
 * sequence: forward, left, right, back, and stop. Each movement lasts 1 second,
 * and the pattern repeats every 5 seconds.
 */
class DroneController : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new DroneController node.
     *
     * Initializes the publisher and sets up the timer for movement pattern execution.
     */
    DroneController();

 private:
    /**
     * @brief Executes the drone movement pattern.
     *
     * Moves the drone forward, turns left, turns right, moves back, and stops,
     * each for 1 second.
     */
    void executeMovementPattern();

    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic execution
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // Publisher for velocity commands
};

