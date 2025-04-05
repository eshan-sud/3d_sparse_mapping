#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <cstdlib>
#include <fstream>
#include <Eigen/Dense>

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
        this->declare_parameter<std::string>("vocabulary_path", "");
        this->declare_parameter<std::string>("settings_path", "");

        // Get parameters
        std::string vocab_path, settings_path;
        this->get_parameter_or("vocabulary_path", vocab_path, std::string(""));
        this->get_parameter_or("settings_path", settings_path, std::string(""));
        // If parameters are not set, use default paths
        if (vocab_path.empty()) {
            std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/home/minor-project";
            vocab_path = home_dir + "/ros2_test/src/ros2_orbslam3_wrapper/ORB_SLAM3/Vocabulary/ORBvoc.txt";
        }
        if (settings_path.empty()) {
            std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/home/minor-project";
            settings_path = home_dir + "/ros2_test/src/ros2_orbslam3_wrapper/ORB_SLAM3/Examples/Monocular/TUM1.yaml"; // TUM1 dataset
            //settings_path = home_dir + "/ros2_test/src/ros2_orbslam3_wrapper/ORB_SLAM3/Examples/Monocular/EuRoC.yaml" // EuRoC dataset
        }

        // Check if files exist
        if (!file_exists(vocab_path) || !file_exists(settings_path)) {
            RCLCPP_ERROR(this->get_logger(), "Required files not found. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Initialise ORB-SLAM3
        // slam_system_ = std::make_shared<ORB_SLAM3::System>(
        //     vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, false
        // );
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true // Show Pangolin Viewer
        );

        // Subscribe to image topic
        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&ORBSLAM3Node::image_callback, this, std::placeholders::_1),
            "raw"
        );

        // Publishers
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/trajectory", 10);
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Node Initialised.");
    }

private:
    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    nav_msgs::msg::Path trajectory_;

    bool file_exists(const std::string& filename) {
        std::ifstream file(filename);
        return file.good();
    }

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

