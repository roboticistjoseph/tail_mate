/**
 * @file template_detection.cpp
 * @author Joseph Katakam (jkatak73@terpmail.umd.edu)
 * @brief ROS2 node that subscribes to a drone's camera feed,
 *        detects a TurtleBot3 follower using hybrid contour-template matching,
 *        and publishes velocity commands to follow the TurtleBot3.
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

class TB3FollowerNode : public rclcpp::Node {
 public:
    TB3FollowerNode() : Node("tb3_follower_node") {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("followme_drone");
        std::string ref_image_path = pkg_share_dir + "/ref_img/tb3_white_cam_view.png";
        ref_img_ = cv::imread(ref_image_path, cv::IMREAD_GRAYSCALE);
        if (ref_img_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load reference image from %s", ref_image_path.c_str());
            throw std::runtime_error("Reference image not found");
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/simple_drone/bottom/image_raw", 10,
            std::bind(&TB3FollowerNode::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "TB3 Follower Node started");
    }

 private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    cv::Mat ref_img_;
    geometry_msgs::msg::Twist velocity_msg_;

    // --- Helper: Rotate image around its center ---
    cv::Mat rotate_image(const cv::Mat& src, double angle) {
        cv::Point2f center(src.cols/2.0F, src.rows/2.0F);
        cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
        cv::Mat dst;
        cv::warpAffine(src, dst, rot, src.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        return dst;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        // --- Try Template Matching (with scaling) ---
        double best_val = -1.0;
        cv::Point best_loc;
        int best_tpl_w = 0, best_tpl_h = 0;
        double best_scale = 1.0;
        // rotation can be added but increasing computaion load, so keeping it simple for robustness
        std::vector<double> scales = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
        for (double scale : scales) {
            cv::Mat scaled_template;
            cv::resize(ref_img_, scaled_template, cv::Size(), scale, scale, cv::INTER_LINEAR);

            if (scaled_template.cols > gray_image.cols || scaled_template.rows > gray_image.rows)
                continue;

            cv::Mat result;
            cv::matchTemplate(gray_image, scaled_template, result, cv::TM_CCOEFF_NORMED);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);

            if (max_val > best_val) {
                best_val = max_val;
                best_loc = max_loc;
                best_tpl_w = scaled_template.cols;
                best_tpl_h = scaled_template.rows;
                best_scale = scale;
            }
        }

        int mid_x = gray_image.cols / 2;
        int mid_y = gray_image.rows / 2;
        int detected_cX = 0, detected_cY = 0;
        bool template_found = (best_val > 0.6);  // Tuned threshold as needed

        // --- Contour Detection (always run, needed for fallback) ---
        cv::Mat binary;
        cv::threshold(gray_image, binary, 200, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find largest contour (assume it's LIDAR)
        double largest_area = 0.0;
        int largest_contour_index = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area) {
                largest_area = area;
                largest_contour_index = static_cast<int>(i);
            }
        }

        int cX = 0, cY = 0;
        bool contour_found = false;
        if (largest_contour_index != -1) {
            cv::Moments M = cv::moments(contours[largest_contour_index]);
            if (M.m00 != 0) {
                cX = static_cast<int>(M.m10 / M.m00);
                cY = static_cast<int>(M.m01 / M.m00);
                contour_found = true;
                cv::drawContours(cv_ptr->image, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);
                cv::circle(cv_ptr->image, cv::Point(cX, cY), 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        // --- Control Logic ---
        if (template_found) {
            // Draw template match rectangle
            cv::rectangle(cv_ptr->image, best_loc,
                cv::Point(best_loc.x + best_tpl_w, best_loc.y + best_tpl_h),
                cv::Scalar(0, 255, 255), 2);

            // Use contour center if it falls inside the template match
            if (contour_found &&
                cX >= best_loc.x && cX <= best_loc.x + best_tpl_w &&
                cY >= best_loc.y && cY <= best_loc.y + best_tpl_h) {
                detected_cX = cX;
                detected_cY = cY;
                RCLCPP_INFO(this->get_logger(), "Template+Contour: Using contour center inside template.");
            } else {
                // Otherwise, use template center
                detected_cX = best_loc.x + best_tpl_w / 2;
                detected_cY = best_loc.y + best_tpl_h / 2;
                RCLCPP_INFO(this->get_logger(), "Template: Using template center.");
            }
        } else if (contour_found) {
            // Fallback: use contour center
            detected_cX = cX;
            detected_cY = cY;
            RCLCPP_WARN(this->get_logger(), "No confident template match, using contour center.");
        } else {
            // No detection
            detected_cX = mid_x;
            detected_cY = mid_y;
            RCLCPP_WARN(this->get_logger(), "No contour or template match found.");
        }

        // Draw image center for reference
        cv::circle(cv_ptr->image, cv::Point(mid_x, mid_y), 5, cv::Scalar(255, 0, 0), -1);

        int error_x = mid_x - detected_cX;
        int error_y = mid_y - detected_cY;

        RCLCPP_INFO(this->get_logger(), "error_x = %d, error_y = %d", error_x, error_y);

        velocity_msg_.linear.x = -error_y * 0.001;
        velocity_msg_.angular.z = error_x * 0.005;

        publisher_->publish(velocity_msg_);

        cv::imshow("Template Matching", cv_ptr->image);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TB3FollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
