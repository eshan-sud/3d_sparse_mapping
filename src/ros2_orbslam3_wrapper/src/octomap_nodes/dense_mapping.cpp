
// TODO : Complete dense mapping using Octomap

// Updated Dense Mapping Node with YAML intrinsics parsing and OctoMap saving/publishing

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap/ColorOcTree.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using sensor_msgs::msg::Image;
using geometry_msgs::msg::PoseStamped;

class DenseColorMappingNode : public rclcpp::Node {
public:
    DenseColorMappingNode() : Node("dense_color_mapping_node") {
        this->declare_parameter<std::string>("settings_path", "");

        std::string settings_path;
        this->get_parameter("settings_path", settings_path);

        if (!load_camera_intrinsics(settings_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load camera intrinsics from YAML");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Loaded intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f depth_scale=%.2f", fx_, fy_, cx_, cy_, depth_scale_);
        }

        using namespace message_filters;
        rgb_sub_.subscribe(this, "/camera/image_raw");
        depth_sub_.subscribe(this, "/camera/image_depth");
        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&DenseColorMappingNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));

        pose_sub_ = this->create_subscription<PoseStamped>(
            "/orb_slam3/camera_pose", 100,
            std::bind(&DenseColorMappingNode::pose_callback, this, std::placeholders::_1));

        octree_ = std::make_shared<octomap::ColorOcTree>(0.03);
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/dense_octomap", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&DenseColorMappingNode::publish_octomap, this));

        RCLCPP_INFO(this->get_logger(), "Dense Color Mapping Node initialized.");
        startup_time_ = this->now();
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<Image, Image> SyncPolicy;
    message_filters::Subscriber<Image> rgb_sub_, depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    std::deque<PoseStamped::ConstSharedPtr> pose_buffer_;
    std::mutex map_mutex_, pose_mutex_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<octomap::ColorOcTree> octree_;
    rclcpp::Time startup_time_;

    float fx_, fy_, cx_, cy_;
    double depth_scale_ = 5000.0;

    bool load_camera_intrinsics(const std::string &yaml_path) {
        cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fx_ = (float)fs["Camera1.fx"];
        fy_ = (float)fs["Camera1.fy"];
        cx_ = (float)fs["Camera1.cx"];
        cy_ = (float)fs["Camera1.cy"];
        depth_scale_ = (double)fs["RGBD.DepthMapFactor"];
        return true;
    }

    void pose_callback(const PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_buffer_.push_back(msg);
        while (pose_buffer_.size() > 100) pose_buffer_.pop_front();
    }

    PoseStamped::ConstSharedPtr get_closest_pose(const rclcpp::Time& stamp) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        PoseStamped::ConstSharedPtr closest = nullptr;
        rclcpp::Duration smallest_diff = rclcpp::Duration::from_seconds(9999.0);

        for (auto& pose : pose_buffer_) {
            rclcpp::Time pose_time(pose->header.stamp);
            rclcpp::Duration diff = (stamp > pose_time) ? (stamp - pose_time) : (pose_time - stamp);
            if (diff < rclcpp::Duration::from_seconds(0.1) && diff < smallest_diff) {
                smallest_diff = diff;
                closest = pose;
            }
        }
        return closest;
    }

    void rgbd_callback(const Image::ConstSharedPtr& rgb_msg,
                       const Image::ConstSharedPtr& depth_msg) {
        if ((this->now() - startup_time_).seconds() < 1.0 || pose_buffer_.empty()) {
            return;
        }

        auto pose_msg = get_closest_pose(rgb_msg->header.stamp);
        if (!pose_msg) return;

        RCLCPP_DEBUG(this->get_logger(), "Matched RGB-D frame timestamp: %.3f with pose timestamp: %.3f",
                     rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9,
                     pose_msg->header.stamp.sec + pose_msg->header.stamp.nanosec * 1e-9);

        cv::Mat rgb_image = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
        cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg)->image;

        std::lock_guard<std::mutex> lock(map_mutex_);

        Eigen::Quaternionf q(
            pose_msg->pose.orientation.w,
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z);
        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Vector3f t(
            pose_msg->pose.position.x,
            pose_msg->pose.position.y,
            pose_msg->pose.position.z);

        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                float depth = 0.0f;
                if (depth_image.type() == CV_16UC1) {
                    depth = static_cast<float>(depth_image.at<uint16_t>(v, u)) / static_cast<float>(depth_scale_);
                } else if (depth_image.type() == CV_32FC1) {
                    depth = depth_image.at<float>(v, u);
                }

                if (!std::isfinite(depth) || depth <= 0.2f || depth >= 5.0f) continue;

                float x = (u - cx_) * depth / fx_;
                float y = (v - cy_) * depth / fy_;
                float z = depth;

                Eigen::Vector3f pt_cam(x, y, z);
                Eigen::Vector3f pt_world = R * pt_cam + t;

                const cv::Vec3b& color = rgb_image.at<cv::Vec3b>(v, u);
                octree_->updateNode(octomap::point3d(pt_world.x(), pt_world.y(), pt_world.z()), true);
                octree_->integrateNodeColor(pt_world.x(), pt_world.y(), pt_world.z(), color[2], color[1], color[0]);
            }
        }

        octree_->updateInnerOccupancy();
    }

    void publish_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        octomap_msgs::msg::Octomap msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        if (octomap_msgs::fullMapToMsg(*octree_, msg)) {
            octomap_pub_->publish(msg);
            octree_->writeBinary("/home/minor-project/ros2_test/results/dense_map.bt");
            RCLCPP_INFO(this->get_logger(), "Published Octomap and saved to dense_map.bt with %zu nodes", octree_->size());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DenseColorMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
