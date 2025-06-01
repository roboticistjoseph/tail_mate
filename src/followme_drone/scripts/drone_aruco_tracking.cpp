/**
 * @file drone_aruco_tracking.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief Implementation of the ArucoFollowerDebug class, which defines Drone's behavior
 *        to follow an ArUco marker using computer vision techniques.
 * @version 0.1
 * @date 2024-05-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// including header files
#include "../include/drone_aruco_tracking.hpp"

/**
 * @brief Constructor for the ArucoFollower class.
 * 
 */
ArucoFollower::ArucoFollower() : Node("aruco_follower_node") {
    // Subscribe to the drone's downward camera image
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/simple_drone/bottom/image_raw", 10,
        std::bind(&ArucoFollower::image_callback, this, std::placeholders::_1));

    // Publisher for drone velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

    // Set up ArUco marker detection
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(this->get_logger(), "Aruco Follower Debug Node started. Looking for marker ID %d.", target_marker_id);
}

/**
 * @brief Callback function for processing incoming images from the drone's camera.
 * 
 * @param msg The incoming image message containing the camera feed.
 */
void ArucoFollower::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    // Detect ArUco markers in the image
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected;
    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, detector_params_, rejected);

    geometry_msgs::msg::Twist cmd_vel;
    bool marker_found = false;

    // Draw all detected markers for debugging
    if (!marker_ids.empty())
        cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

    for (size_t i = 0; i < marker_ids.size(); ++i) {
        if (marker_ids[i] == target_marker_id) {
            marker_found = true;
            // Compute centroid of the marker
            cv::Point2f centroid(0, 0);
            for (const auto& pt : marker_corners[i]) centroid += pt;
            centroid *= 0.25f;

            // Draw centroid and image center for visual debugging
            cv::circle(image, centroid, 6, cv::Scalar(0, 0, 255), -1);
            cv::Point2f img_center(image.cols / 2.0f, image.rows / 2.0f);
            cv::circle(image, img_center, 6, cv::Scalar(255, 0, 0), 2);
            cv::arrowedLine(image, img_center, centroid, cv::Scalar(0, 255, 255), 2);

            // Calculate error between marker centroid and image center
            double error_x = centroid.x - img_center.x;  // left/right
            double error_y = centroid.y - img_center.y;  // forward/backward

            // Proportional control for movement
            cmd_vel.linear.x = -Kp_y * error_y;  // Forward/backward
            cmd_vel.linear.y = -Kp_x * error_x;  // Left/right

            // Proportional control for yaw (head turning)
            cmd_vel.angular.z = -Kp_yaw * error_x;  // Turn to face marker

            cmd_vel.linear.z = 0.0;

            // === Update last known velocities ===
            last_linear_x_ = cmd_vel.linear.x;
            last_linear_y_ = cmd_vel.linear.y;
            last_linear_z_ = cmd_vel.linear.z;
            last_angular_z_ = cmd_vel.angular.z;

            RCLCPP_INFO(this->get_logger(),
                "Marker ID %d found at (%.1f, %.1f). Error: (x=%.1f, y=%.1f), Yaw cmd: %.3f",
                target_marker_id, centroid.x, centroid.y, error_x, error_y, cmd_vel.angular.z);

            break;  // Only follow the first found marker with ID 3
        }
    }

    if (!marker_found) {
        // === Fallback: Continue in last known direction if moving, else stay in place ===
        bool was_moving = (std::abs(last_linear_x_) > 1e-4) ||
                          (std::abs(last_linear_y_) > 1e-4) ||
                          (std::abs(last_linear_z_) > 1e-4) ||
                          (std::abs(last_angular_z_) > 1e-4);

        if (was_moving) {
            // Continue moving in last known direction
            cmd_vel.linear.x = last_linear_x_;
            cmd_vel.linear.y = last_linear_y_;
            cmd_vel.linear.z = last_linear_z_;
            cmd_vel.angular.z = last_angular_z_;

            RCLCPP_WARN(this->get_logger(),
                "Marker ID %d NOT found. Continuing in last known direction: "
                "linear.x=%.3f, linear.y=%.3f, linear.z=%.3f, angular.z=%.3f",
                target_marker_id, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z);
        } else {
            // Stay in place
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.z = 0.0;

            RCLCPP_WARN(this->get_logger(),
                "Marker ID %d NOT found. Drone was stationary, so staying in place.",
                target_marker_id);
            }
    }
    // // Fallback: rotate in place or move in a small circle to search for the marker
    // cmd_vel.linear.x = fallback_linear_x;   // Set to 0 for pure rotation, >0 for circle
    // cmd_vel.linear.y = 0.0;
    // cmd_vel.linear.z = 0.0;
    // cmd_vel.angular.z = fallback_angular_z;

    // RCLCPP_WARN(this->get_logger(),
    //     "Marker ID %d NOT found. Executing circular search: linear.x=%.2f, angular.z=%.2f",
    //     target_marker_id, cmd_vel.linear.x, cmd_vel.angular.z);

    // Publish velocity command to the drone
    publisher_->publish(cmd_vel);

    // Drone camera feedback
    cv::imshow("Aruco Tracking", image);
    cv::waitKey(1);
}

/**
 * @brief Main function: Initializes ROS2, creates the ArUco follower node, and spins it.
 * @param argc Argument count
 * @param argv Argument vector
 * @return Exit status
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoFollower>());
    rclcpp::shutdown();
    return 0;
}
