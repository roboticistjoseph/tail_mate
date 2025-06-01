/**
 * @file contour_detection.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief ROS2 node that subscribes to a drone's camera feed,
 *        detects white "contours" (specified bountary area) in the image, and publishes velocity commands
 *        to follow the largest detected contour.
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

/**
 * @brief ContourDetection class
 * 
 */
class ContourDetection : public rclcpp::Node {
 public:
    ContourDetection() : Node("camera_subscriber_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/simple_drone/bottom/image_raw", 10,
            std::bind(&ContourDetection::camera_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "------ Contours Node Started -----");
    }

 private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist velocity_msg_;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr camera_msg){
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(camera_msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat binary;
        cv::threshold(gray_image, binary, 200, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        RCLCPP_INFO(this->get_logger(), "Contours found: %zu", contours.size());

        // Define min and max contour area thresholds (tune these)
        const double min_area = 50.0;    // Ignore too small objects
        const double max_area = 1000.0;  // Ignore too large objects

        double largest_area = 0.0;
        int largest_contour_index = -1;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > min_area && area < max_area && area > largest_area) {
                largest_area = area;
                largest_contour_index = static_cast<int>(i);
            }
        }

        int cX = 0, cY = 0;
        if (largest_contour_index != -1) {
            cv::Moments M = cv::moments(contours[largest_contour_index]);
            if (M.m00 != 0) {
                cX = static_cast<int>(M.m10 / M.m00);
                cY = static_cast<int>(M.m01 / M.m00);

                cv::drawContours(cv_ptr->image, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);
                cv::circle(cv_ptr->image, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        int mid_x = gray_image.cols / 2;
        int mid_y = gray_image.rows / 2;
        cv::circle(cv_ptr->image, cv::Point(mid_x, mid_y), 5, cv::Scalar(255, 0, 0), -1);

        int error_x = mid_x - cX;
        int error_y = mid_y - cY;

        RCLCPP_INFO(this->get_logger(), "error_x = %d, error_y = %d", error_x, error_y);

        velocity_msg_.linear.x = -error_y * 0.001;
        velocity_msg_.angular.z = error_x * 0.005;

        publisher_->publish(velocity_msg_);

        cv::imshow("Contour Detection", cv_ptr->image);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContourDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
