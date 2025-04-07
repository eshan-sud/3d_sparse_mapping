// src/depth_estimator_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>

class DepthEstimator : public rclcpp::Node {
public:
    DepthEstimator() : Node("depth_estimator") {
        using std::placeholders::_1;
        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&DepthEstimator::image_callback, this, _1),
            "raw"
        );

        depth_pub_ = image_transport::create_publisher(this, "/camera/depth_estimated");
        RCLCPP_INFO(this->get_logger(), "Depth Estimator Node started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat input_image = cv_ptr->image;

            // Perform depth estimation (dummy example)
            cv::Mat depth_map(input_image.rows, input_image.cols, CV_32FC1, cv::Scalar(1.0));

            auto out_msg = cv_bridge::CvImage(msg->header, "32FC1", depth_map).toImageMsg();
            depth_pub_.publish(out_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher depth_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthEstimator>());
    rclcpp::shutdown();
    return 0;
}
