#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <System.h>
#include <cstdlib>

class ORBSLAM3Node : public rclcpp::Node {
public:
    ORBSLAM3Node() : Node("orb_slam3_node") {

        // Declare Parameters
        std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/home/minor-project"; // Get HOME directory
        std::string vocab_path = home_dir + "/ros2_test/src/ros2_orbslam3_wrapper/ORB_SLAM3/Vocabulary/ORBvoc.txt";
        std::string settings_path = home_dir + "/ros2_test/src/ros2_orbslam3_wrapper/ORB_SLAM3/Examples/Monocular/EuRoC.yaml";
        this->declare_parameter<std::string>("vocabulary_path", vocab_path);
        this->declare_parameter<std::string>("settings_path", settings_path);

        // Get parameters
        vocab_path = this->get_parameter("vocabulary_path").as_string();
        settings_path = this->get_parameter("settings_path").as_string();

        // Initialize ORB-SLAM3 System
        slam_system_ = std::make_shared<ORB_SLAM3::System>(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

        // Subscribe to image topic
        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&ORBSLAM3Node::image_callback, this, std::placeholders::_1),
            "raw");

        //image_sub_ = image_transport::create_subscription(
        //    this, "/camera/image_raw",
        //    std::bind(&ORBSLAM3Node::image_callback, this, std::placeholders::_1),
        //    "raw");

        RCLCPP_INFO(this->get_logger(), "ORB SLAM3 ROS2 Wrapper Started with Vocabulary: %s and Settings: %s", vocab_path.c_str(), settings_path.c_str());
        //std::string vocab_path = "/path/to/ORBvoc.txt";
        //std::string settings_path = "/path/to/camera.yaml";
        //slam_system_ = std::make_shared<ORB_SLAM3::System>(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }
        double timestamp = this->now().seconds(); // Convert to seconds
        RCLCPP_INFO(this->get_logger(), "ORB SLAM3 processing image at timestamp: %f", timestamp);
        slam_system_->TrackMonocular(cv_ptr->image, timestamp);
        //slam_system_->TrackMonocular(cv_ptr->image, this->now().nanoseconds()); // ORIGINAL
    }

    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBSLAM3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

