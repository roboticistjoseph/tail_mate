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

/**
 * @file drone_controller.cpp
 * @brief Implementation of the DroneController class for ROS2 drone movement control.
 */

DroneController::DroneController() : Node("drone_controller") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&DroneController::executeMovementPattern, this));
}

void DroneController::executeMovementPattern() {
    auto message = geometry_msgs::msg::Twist();

    // Obtaining Height
    message.linear.z = 1.0;

    // Forward
    message.linear.x = 5.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
    rclcpp::sleep_for(1s);

    // Left
    message.linear.x = 0.0;
    message.angular.z = 5.0;
    publisher_->publish(message);
    rclcpp::sleep_for(1s);

    // Forward
    message.linear.x = 5.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
    rclcpp::sleep_for(1s);

    // Right
    message.angular.z = -1.0;
    publisher_->publish(message);
    rclcpp::sleep_for(1s);

    // Back
    message.angular.z = 0.0;
    message.linear.x = -1.0;
    publisher_->publish(message);
    rclcpp::sleep_for(1s);

    // Stop
    message.linear.x = 0.0;
    publisher_->publish(message);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
