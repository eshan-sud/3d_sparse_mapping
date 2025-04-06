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
#include <fstream>

std::atomic<bool> running(true);  // Global flag to handle shutdown properly

namespace fs = std::filesystem;

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher(const std::string& config_path) : Node("camera_publisher"), image_count_(0) {
        cv::FileStorage fs(config_path, cv::FileStorage::READ); // Load the YAML config file
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
        std::string rgb_dir_ = output_dir_ + "/rgb";
        std::string rgb_txt_path_ = output_dir_ + "/rgb.txt";
        fs::create_directories(output_dir_);
        fs::create_directories(rgb_dir_);

        rgb_txt_file_.open(rgb_txt_path_, std::ios::out);
        if (rgb_txt_file_.is_open()) {
            rgb_txt_file_ << "# color images\n";
            rgb_txt_file_ << "# file: 'real-time'\n";
            rgb_txt_file_ << "# timestamp filename\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create rgb.txt at %s", rgb_txt_path_.c_str());
        }

        // Create publisher & timer
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
            std::bind(&CameraPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully.");
    }

    ~CameraPublisher() {
        if (rgb_txt_file_.is_open()) {
            rgb_txt_file_.close();
        }
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
        // NEWWWW
        // Convert frame to ROS 2 Image message
        auto now = this->get_clock()->now();
        double timestamp = now.seconds();

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = now;
        msg->header.frame_id = "camera_frame";
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = frame.step;
        msg->data.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
        publisher_->publish(std::move(msg));

        std::ostringstream filename_stream;
        filename_stream << std::fixed << std::setprecision(6) << timestamp;
        std::string filename = filename_stream.str();
        std::string filepath = rgb_dir_ + "/" + filename + ".png";
        cv::imwrite(filepath, frame);

        if (rgb_txt_file_.is_open()) {
            rgb_txt_file_ << filename << " rgb/" << filename << ".png\n";
        }

        RCLCPP_INFO(this->get_logger(), "%d Images Published", image_count_++);

        // Cap imshow to 10 FPS
        auto now_display = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_display - last_display_time_).count();
        if (elapsed_ms >= 1000 / display_fps_cap_) {
            cv::imshow("Camera Feed", frame);
            cv::waitKey(1);
            last_display_time_ = now_display;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture capture_;
    int image_count_;
    int fps_, width_, height_, rgb_mode_, device_index_;
    std::string output_dir_;
    std::string rgb_dir_;
    std::string rgb_txt_path_;
    std::ofstream rgb_txt_file_;

    std::chrono::steady_clock::time_point last_display_time_;
    const int display_fps_cap_ = 10; // Cap OpenCV imshow to 10 FPS
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
    std::signal(SIGINT, signal_handler); // Handle Ctrl+C (SIGINT)

    // Use `spin_some()` to allow OpenCV to refresh, with a small sleep to reduce CPU usage
    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Prevent high CPU usage
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    node.reset();  // Ensure the node is destroyed before releasing OpenCV resources
    cv::destroyAllWindows();  // Close OpenCV windows

    return 0;
}
