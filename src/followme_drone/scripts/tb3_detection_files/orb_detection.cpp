/**
 * @file orb_detection.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief ROS2 node that subscribes to a drone's bottom camera feed,
 *        detects the TurtleBot using ORB feature matching, and publishes velocity commands
 *        to follow the TurtleBot.
 * @version 0.1
 * @date 2024-05-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
// including header files
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class ORBTrackerNode : public rclcpp::Node {
 public:
    ORBTrackerNode() : Node("orb_tracker_node") {
        // Load reference image of TurtleBot (grayscale)
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("followme_drone");
        std::string ref_image_path = pkg_share_dir + "/ref_img/tb3_white_cam_view.png";

        ref_img_ = cv::imread(ref_image_path, cv::IMREAD_GRAYSCALE);
        if (ref_img_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load reference image from %s", ref_image_path.c_str());
            throw std::runtime_error("Reference image not found");
        }

        // Initialize ORB detector and descriptor
        orb_ = cv::ORB::create();

        // Detect keypoints and descriptors in reference image once
        orb_->detectAndCompute(ref_img_, cv::noArray(), ref_keypoints_, ref_descriptors_);
        if (ref_descriptors_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No descriptors found in reference image");
            throw std::runtime_error("No descriptors in reference image");
        }

        // Create BFMatcher with Hamming distance for ORB
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);

        // Subscribe to drone bottom camera image
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/simple_drone/bottom/image_raw", 10,
            std::bind(&ORBTrackerNode::image_callback, this, std::placeholders::_1));

        // Publisher for drone velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "ORB Tracker Node started");
    }

 private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    cv::Mat ref_img_;
    std::vector<cv::KeyPoint> ref_keypoints_;
    cv::Mat ref_descriptors_;
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;

    geometry_msgs::msg::Twist velocity_msg_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert current frame to grayscale
        cv::Mat frame_gray;
        cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);

        // Detect ORB keypoints and descriptors in current frame
        std::vector<cv::KeyPoint> frame_keypoints;
        cv::Mat frame_descriptors;
        orb_->detectAndCompute(frame_gray, cv::noArray(), frame_keypoints, frame_descriptors);

        if (frame_descriptors.empty()) {
            RCLCPP_WARN(this->get_logger(), "No descriptors found in current frame");
            return;
        }

        // Match descriptors between reference and current frame
        std::vector<cv::DMatch> matches;
        matcher_->match(ref_descriptors_, frame_descriptors, matches);

        if (matches.empty()) {
            RCLCPP_WARN(this->get_logger(), "No matches found");
            return;
        }

        // Filter good matches based on distance
        double max_dist = 0; double min_dist = 100;
        for (const auto& m : matches) {
            double dist = m.distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        std::vector<cv::DMatch> good_matches;
        for (const auto& m : matches) {
            if (m.distance <= std::max(2 * min_dist, 30.0)) {
                good_matches.push_back(m);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Good matches: %zu", good_matches.size());

        if (good_matches.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Not enough good matches to track");
            return;
        }

        // Compute centroid of matched keypoints in current frame
        cv::Point2f centroid(0, 0);
        for (const auto& m : good_matches) {
            centroid += frame_keypoints[m.trainIdx].pt;
        }
        centroid *= (1.0f / good_matches.size());

        // Draw matches and centroid for visualization
        cv::Mat img_matches;
        cv::drawMatches(ref_img_, ref_keypoints_, frame_gray, frame_keypoints, good_matches, img_matches,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        cv::circle(img_matches, centroid + cv::Point2f(static_cast<float>(ref_img_.cols), 0),
                    5, cv::Scalar(0, 0, 255), -1);

        cv::imshow("ORB Detection", img_matches);
        cv::waitKey(1);

        // Calculate error relative to image center
        int mid_x = frame_gray.cols / 2;
        int mid_y = frame_gray.rows / 2;
        int error_x = mid_x - static_cast<int>(centroid.x);
        int error_y = mid_y - static_cast<int>(centroid.y);

        RCLCPP_INFO(this->get_logger(), "error_x = %d, error_y = %d", error_x, error_y);

        // Control logic: adjust linear and angular velocities based on error
        velocity_msg_.linear.x = -error_y * 0.001;
        velocity_msg_.angular.z = error_x * 0.005;

        publisher_->publish(velocity_msg_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
