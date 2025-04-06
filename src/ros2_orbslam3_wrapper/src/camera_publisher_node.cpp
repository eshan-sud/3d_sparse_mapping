#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/persistence.hpp>
#include <cv_bridge/cv_bridge.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <csignal>  // For handling Ctrl+C (SIGINT)
#include <filesystem> // C++17 required

std::atomic<bool> running(true);  // Global flag to handle shutdown properly

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher(const std::string& config_path) : Node("camera_publisher"), image_count_(0) {
        // Load YAML using OpenCV
        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", config_path.c_str());
            return;
        }

        // Read parameters from ORB-SLAM3-style YAML
        fs["Camera.fps"] >> fps_;
        fs["Camera.width"] >> width_;
        fs["Camera.height"] >> height_;
        fs["Camera.RGB"] >> rgb_mode_;

        // Check if "Camera.device_index" exists in YAML
        if (fs["Camera.device_index"].empty()) {
            RCLCPP_WARN(this->get_logger(), "No 'Camera.device_index' found in YAML. Defaulting to 0.");
            device_index_ = 0;
        } else {
            fs["Camera.device_index"] >> device_index_;
        }

        // Open camera
        capture_.open(device_index_);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera device: %d", device_index_);
            return;
        }
        std::string output_dir_ = "/home/minor-project/minor-project/Datasets/real-time"; // Set output directory
        std::filesystem::create_directories(output_dir_);

        // Create publisher & timer
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
            std::bind(&CameraPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully.");
    }

private:
    void timer_callback() {
        if (!capture_.isOpened()) {
            return;
        }
        cv::Mat frame;
        capture_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }
        // Convert frame to ROS 2 Image message
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = frame.step;
        msg->data.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
        publisher_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "%d Images Published", image_count_++);
        std::ostringstream filename;
        filename << output_dir_ << "/frame_" << std::setw(6) << std::setfill('0') << image_count_ << ".png";
        cv::imwrite(filename.str(), frame);
        // Show image using OpenCV
        cv::imshow("Camera Feed", frame);
        cv::waitKey(1);  // Required to refresh the OpenCV window
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture capture_;
    int image_count_;

    // Declare missing parameters
    int fps_;
    int width_;
    int height_;
    int rgb_mode_;
    int device_index_;
};

// Signal handler for SIGINT (Ctrl+C)
void signal_handler(int signum) {
    running = false;  // Set global flag to stop the main loop
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Ensure a config file path is provided
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_publisher"), "Usage: ros2 run ros2_orbslam3_wrapper camera_publisher_node <config_file.yaml>");
        return 1;
    }

    std::string config_path = argv[1];

    auto node = std::make_shared<CameraPublisher>(config_path);

    // Handle Ctrl+C (SIGINT)
    std::signal(SIGINT, signal_handler);

    // Use `spin_some()` to allow OpenCV to refresh, with a small sleep to reduce CPU usage
    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Prevent high CPU usage
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();

    // Explicitly release the camera
    node.reset();  // Ensure the node is destroyed before releasing OpenCV resources
    cv::destroyAllWindows();  // Close OpenCV windows

    return 0;
}
