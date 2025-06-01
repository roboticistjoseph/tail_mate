/**
 * @file drone_aruco_tracking.hpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Header file for the ArucoFollowerDebug class, which defines Drone's behavior
 *        to follow an ArUco marker using computer vision techniques.
 * @version 0.1
 * @date 2024-05-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// Guard to prevent multiple inclusions
#ifndef SRC_FOLLOWME_DRONE_INCLUDE_DRONE_ARUCO_TRACKING_HPP_
#define SRC_FOLLOWME_DRONE_INCLUDE_DRONE_ARUCO_TRACKING_HPP_

// Standard C++ libraries
#include <memory>
// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
// OpenCV libraries for image processing and ArUco marker detection
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"

/**
 * @class ArucoFollowerDebug
 * @brief ROS2 node for following an ArUco marker with a drone.
 *
 * Subscribes to a camera image, detects a specific ArUco marker,
 * and publishes velocity commands to follow it. If the marker is lost,
 * the drone continues in its last known direction (if it was moving),
 * or stays in place if it was already stationary.
 */
class ArucoFollower : public rclcpp::Node {
 public:
    /**
     * @brief Constructor: Initializes node, subscriptions, publishers, and detector.
     */
    ArucoFollower();

 private:
    // === ROS2 Communication ===
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  // Image subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;      // Velocity command publisher

    // === ArUco Detection ===
    cv::Ptr<cv::aruco::Dictionary> dictionary_;                // ArUco dictionary
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;   // Detection parameters
    const int target_marker_id = 3;  // ID of the ArUco marker to follow

    // === PID Control Parameters ===
    const double Kp_x = 0.0025;      // Proportional gain for lateral (y) control
    const double Kp_y = 0.0025;      // Proportional gain for forward/back (x) control
    const double Kp_yaw = 0.005;     // Proportional gain for yaw (angular z) control

    // === Fallback Search Parameters ===
    const double fallback_angular_z = 0.3;  // Angular speed for search (rad/s)
    const double fallback_linear_x = 0.1;   // Linear speed for search (m/s)

    // === Last Known Velocity State ===
    double last_linear_x_ = 0.0;   ///< Last known linear.x velocity
    double last_linear_y_ = 0.0;   ///< Last known linear.y velocity
    double last_linear_z_ = 0.0;   ///< Last known linear.z velocity
    double last_angular_z_ = 0.0;  ///< Last known angular.z velocity

    /**
     * @brief Image callback: Detects marker, computes control, and publishes velocity.
     *        If marker is lost, continues in last known direction or stays in place.
     * @param msg Incoming image message from the drone's camera.
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif  // SRC_FOLLOWME_DRONE_INCLUDE_DRONE_ARUCO_TRACKING_HPP_
