
// Monocular ORB-SLAM3 execution node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <cstdlib>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <Tracking.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class ORBSLAM3Node : public rclcpp::Node {
public:
    ORBSLAM3Node() : Node("orb_slam3_node") {

        // Declare Parameters
        this->declare_parameter<std::string>("input_mode", "live"); // live or Pre-recorded / Dataset
        this->declare_parameter<std::string>("vocabulary_path", ""); // Vocabulary file
        this->declare_parameter<std::string>("settings_path", ""); // Settings(YAML) file
        this->declare_parameter<std::string>("dataset_type", "TUM");
        this->declare_parameter<std::string>("dataset_path", "");

        // Get parameters
        std::string vocab_path, settings_path, input_mode, dataset_type, dataset_path;
        this->get_parameter_or("input_mode", input_mode, std::string("live"));  // default: live
        this->get_parameter_or("vocabulary_path", vocab_path, std::string(""));
        this->get_parameter_or("settings_path", settings_path, std::string(""));
        this->get_parameter_or("dataset_type", dataset_type, std::string("TUM"));  // default: TUM
        this->get_parameter_or("dataset_path", dataset_path, std::string(""));

        // Set default paths
        std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/home/minor-project";
        if (vocab_path.empty()) {
            vocab_path = home_dir + "/ros2_test/src/ORB_SLAM3/Vocabulary/ORBvoc.txt";
        }
        if (settings_path.empty()) {
            if (input_mode == "recorded") {
                if (dataset_type == "TUM") {
                    settings_path = home_dir + "/ros2_test/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml";
                } else if (dataset_type == "EuRoC") {
                    settings_path = home_dir + "/ros2_test/src/ORB_SLAM3/Examples/Monocular/EuRoC.yaml";
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown dataset_type: %s. Defaulting to TUM.", dataset_type.c_str());
                    settings_path = home_dir + "/ros2_test/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml";
                }
            } else {
                settings_path = home_dir + "/minor-project/camera-calibration/calibration/my_camera.yaml";
            }
        }

        if (input_mode != "live" && input_mode != "recorded") {
            RCLCPP_ERROR(this->get_logger(), "Invalid input_mode: %s. Choose 'live' or 'recorded'.", input_mode.c_str());
            rclcpp::shutdown();
            return;
        }

        if (!file_exists(vocab_path) || !file_exists(settings_path)) {
            RCLCPP_ERROR(this->get_logger(), "Required files not found. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Initialise ORB-SLAM3
        //slam_system_ = std::make_shared<ORB_SLAM3::System>(
        //    vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, false // Viewer is turned off
        //);
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true
        );

        RCLCPP_INFO(this->get_logger(), "Vocabulary path: %s", vocab_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings path: %s", settings_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Input mode: %s", input_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "Dataset type: %s", dataset_type.c_str());
        if (input_mode == "recorded" && !dataset_path.empty()) {
            if (dataset_path.empty()) {
                RCLCPP_WARN(this->get_logger(), "Dataset path not provided. Please specify a dataset path.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Running ORB-SLAM3 using dataset at: %s", dataset_path.c_str());
            std::thread dataset_thread([this, dataset_type, dataset_path]() {
                this->run_dataset(dataset_type, dataset_path);
                rclcpp::shutdown();
            });
            dataset_thread.detach();
        } else {
            std::string topic_name = "/camera/image_raw";
            RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
            image_sub_ = image_transport::create_subscription(
                this, topic_name,
                std::bind(&ORBSLAM3Node::image_callback, this, std::placeholders::_1),
                "raw"
            );
        }
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/trajectory", 10);
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                publish_trajectory();
                publish_map_points();
            }
        );

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Node Initialised.");
    }


private:
    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    nav_msgs::msg::Path trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool file_exists(const std::string& filename) {
        std::ifstream file(filename);
        return file.good();
    }

    void run_dataset(const std::string &dataset_type, const std::string &dataset_path) {
        if (dataset_type == "EuRoC") {
            // Handle EuRoC dataset (with data.csv)
            std::string assoc_file = dataset_path + "/mav0/cam0/data.csv"; // EuRoC format
            std::ifstream file(assoc_file);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open data.csv in dataset path: %s", dataset_path.c_str());
                return;
            }
            std::string line;
            while (std::getline(file, line) && rclcpp::ok()) {
                if (line.empty() || line[0] == '#') continue; // Skip empty or comment lines

                std::istringstream iss(line);
                double timestamp;
                std::string image_filename;
                iss >> timestamp >> image_filename;

                std::string full_image_path = dataset_path + "mav0/cam0/" + image_filename;
                std::cout << "Loading image from: " << full_image_path << std::endl;  // For debugging
                cv::Mat image = cv::imread(full_image_path, cv::IMREAD_UNCHANGED);
                if (image.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", full_image_path.c_str());
                    continue;
                }
                // Process the image frame with timestamp
                slam_system_->TrackMonocular(image, timestamp);
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Sleep to simulate real-time processing
                // Publish trajectory and map points
                publish_trajectory();
                publish_map_points();
            }
            RCLCPP_INFO(this->get_logger(), "Dataset processing complete.");
        } else if (dataset_type == "TUM") {
            // Handle TUM dataset (with rgb.txt)
            std::string assoc_file = dataset_path + "/rgb.txt"; // TUM format
            std::ifstream file(assoc_file);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open rgb.txt in dataset path: %s", dataset_path.c_str());
                return;
            }
            std::string line;
            while (std::getline(file, line) && rclcpp::ok()) {
                if (line.empty() || line[0] == '#') continue; // Skip empty or comment lines
                std::istringstream iss(line);
                double timestamp;
                std::string image_filename;
                iss >> timestamp >> image_filename;
                std::string full_image_path = dataset_path + "/" + image_filename;
                cv::Mat image = cv::imread(full_image_path, cv::IMREAD_UNCHANGED);
                if (image.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", full_image_path.c_str());
                    continue;
                }
                // Process the image frame with timestamp
                slam_system_->TrackMonocular(image, timestamp);
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Sleep to simulate real-time processing
                // Publish trajectory and map points
                publish_trajectory();
                publish_map_points();
            }
            RCLCPP_INFO(this->get_logger(), "Dataset processing complete.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported dataset type: %s", dataset_type.c_str());
        }
    }

//    void run_dataset(const std::string &dataset_path) {
//        std::string assoc_file = dataset_path + "/rgb.txt"; // Timestamp file for EuRoC
//        std::ifstream file(assoc_file);
//        if (!file.is_open()) {
//            RCLCPP_ERROR(this->get_logger(), "Failed to open rgb.txt in dataset path: %s", dataset_path.c_str());
//            return;
//        }
//        std::string line;
//        while (std::getline(file, line) && rclcpp::ok()) {
//            if (line.empty() || line[0] == '#') continue; // Skip empty or comment lines
//            std::istringstream iss(line);
//            double timestamp;
//            std::string image_filename;
//            iss >> timestamp >> image_filename;

//            std::string full_image_path = dataset_path + "/" + image_filename;
//            cv::Mat image = cv::imread(full_image_path, cv::IMREAD_UNCHANGED);
//            if (image.empty()) {
//                RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", full_image_path.c_str());
//                continue;
//            }
            // Process the image frame with timestamp
//            slam_system_->TrackMonocular(image, timestamp);
//            std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Sleep to simulate real-time processing
            // Publish trajectory and map points
//            publish_trajectory();
//            publish_map_points();
//        }
//        RCLCPP_INFO(this->get_logger(), "Dataset processing complete.");
//    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        if (!slam_system_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
            return;
        }

        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        //double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        slam_system_->TrackMonocular(cv_ptr->image, timestamp);

        publish_trajectory();
        publish_map_points();
    }

    void publish_trajectory() {
        if (!slam_system_) return;

        int tracking_state = slam_system_->GetTrackingState();
        if (tracking_state == ORB_SLAM3::Tracking::LOST) {
            RCLCPP_WARN(this->get_logger(), "Tracking lost. Clearing trajectory.");
            trajectory_.poses.clear();
            return;
        }

        Sophus::SE3f cam_pose = slam_system_->GetTracker()->GetCameraPose();
        Eigen::Matrix4f pose_matrix = cam_pose.matrix();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = pose_matrix(0, 3);
        pose_msg.pose.position.y = pose_matrix(1, 3);
        pose_msg.pose.position.z = pose_matrix(2, 3);
        Eigen::Quaternionf q(Eigen::Matrix3f(pose_matrix.block<3,3>(0,0)));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        trajectory_.header = pose_msg.header;
        trajectory_.poses.push_back(pose_msg);
        trajectory_pub_->publish(trajectory_);
    }

    void publish_map_points() {
        if (!slam_system_) return;

        std::vector<ORB_SLAM3::MapPoint*> map_points = slam_system_->GetTrackedMapPoints();

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(map_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& mp : map_points) {
            if (!mp || mp->isBad()) continue;
            Eigen::Vector3f pos = mp->GetWorldPos();
            *iter_x = pos.x(); ++iter_x;
            *iter_y = pos.y(); ++iter_y;
            *iter_z = pos.z(); ++iter_z;
        }

        map_points_pub_->publish(cloud_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBSLAM3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
