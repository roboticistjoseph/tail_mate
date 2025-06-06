/**
 * @file img_thresholding.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief ROS2 node that subscribes to a drone's camera feed,
 *        detects white color in the image, and publishes velocity commands
 *        to follow the center of detected color.
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
 * @brief ColorDetection class
 * 
 */
class ColorDetection : public rclcpp::Node {
 public:
    ColorDetection() : Node("color_detection_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/simple_drone/bottom/image_raw", 10,
            std::bind(&ColorDetection::camera_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);
        auto velocity_msg = geometry_msgs::msg::Twist();
        RCLCPP_INFO(this->get_logger(), "------ Node Started -----");
    }

 private:
    int cX, cY, mid_x, mid_y, error_x, error_y = 0;
    geometry_msgs::msg::Twist velocity_msg = geometry_msgs::msg::Twist();
    int largest_contour_index = 0;
    double largest_area = 0.0;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr camera_msg) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(camera_msg, "bgr8");
        cv::Mat gray_image, binary;
        // Flipping
        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        // Binary Image
        // detects white color
        cv::threshold(gray_image, binary, 200, 255, cv::THRESH_BINARY);
        // detects black color
        // cv::threshold(gray_image, binary, 50, 255, cv::THRESH_BINARY_INV);

        // Find Contour
        std::vector<std::vector<cv::Point> > contour;
        cv::findContours(binary, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "contours.size() = %d", static_cast<int>(contour.size()));

        // Draw Contour
        cv::drawContours(cv_ptr->image, contour, -1, cv::Scalar(0, 255, 0), 2);

        // Draw Center of Contours with Moment
        for (int i = 0; i < static_cast<int>(contour.size()); i++) {
            double area = cv::contourArea(contour[i]);  // Calculate the area of the contour
            if (area > largest_area) {
                largest_area = area;
                largest_contour_index = i;
            }
        }
        if ( contour.size() ) {  // made change here
            cv::Moments M = cv::moments(contour[largest_contour_index]);
            cX = static_cast<int>(M.m10 / M.m00);
            cY = static_cast<int>(M.m01 / M.m00);
            cv::circle(cv_ptr->image, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
        }
        // Error Calculation
        mid_x = gray_image.cols/2;
        mid_y = gray_image.rows/2;
        cv::circle(cv_ptr->image, cv::Point(mid_x, mid_y), 5, cv::Scalar(255, 0, 0), -1);
        error_x = mid_x - cX;
        error_y = mid_y - cY;

        RCLCPP_INFO(this->get_logger(), "error_x = %d error_y = %d", error_x, error_y);

        // Error Reaction

        //  linear Motion
        velocity_msg.linear.x = -error_y * 0.001;
        // Angular motion
        velocity_msg.angular.z = error_x *0.005;

        // Sendign Velocity
        publisher_->publish(velocity_msg);


        cv::imshow("Color Detection", cv_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetection>());
    rclcpp::shutdown();
    return 0;
}
