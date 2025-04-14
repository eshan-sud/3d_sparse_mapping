
// TODO : Complete dense mapping using Octomap

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap/octomap.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <memory>

class DenseOctoMapMappingNode : public rclcpp::Node {
public:
    DenseOctoMapMappingNode() : Node("dense_octomap_mapping_node") {
        // Subscribe to depth image
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&DenseOctoMapMappingNode::depth_callback, this, std::placeholders::_1)
        );

        // Subscribe to camera pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/camera_pose", 10,
            std::bind(&DenseOctoMapMappingNode::pose_callback, this, std::placeholders::_1)
        );

        // OctoMap resolution: 5 cm
        octree_ = std::make_shared<octomap::OcTree>(0.05);

        // Publisher for visualization
        octomap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dense_octomap", 10);

        // Timer to publish OctoMap
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&DenseOctoMapMappingNode::publish_octomap, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dense OctoMap Mapping Node initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex map_mutex_;
    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped latest_pose_;
    bool pose_received_ = false;

    // Pose callback
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
        pose_received_ = true;
    }

    // Depth callback
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!pose_received_) {
            RCLCPP_WARN(this->get_logger(), "No camera pose received yet.");
            return;
        }

        // Convert sensor_msgs::msg::Image to cv::Mat manually
        cv::Mat depth_image;
        if (msg->encoding == "16UC1") {
            depth_image = cv::Mat(msg->height, msg->width, CV_16UC1, const_cast<uint8_t*>(msg->data.data()), msg->step);
        } else if (msg->encoding == "32FC1") {
            depth_image = cv::Mat(msg->height, msg->width, CV_32FC1, const_cast<uint8_t*>(msg->data.data()), msg->step);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

        // Camera intrinsics (adjust as per your camera)
        float fx = 525.0f;
        float fy = 525.0f;
        float cx = msg->width / 2.0f;
        float cy = msg->height / 2.0f;

        geometry_msgs::msg::PoseStamped pose_copy;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose_copy = latest_pose_;
        }

        Eigen::Quaternionf q(
            pose_copy.pose.orientation.w,
            pose_copy.pose.orientation.x,
            pose_copy.pose.orientation.y,
            pose_copy.pose.orientation.z
        );
        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Vector3f t(
            pose_copy.pose.position.x,
            pose_copy.pose.position.y,
            pose_copy.pose.position.z
        );

        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                float depth;
                if (msg->encoding == "16UC1") {
                    uint16_t d = depth_image.at<uint16_t>(v, u);
                    depth = static_cast<float>(d) / 1000.0f; // convert mm to meters
                } else {
                    depth = depth_image.at<float>(v, u);
                }

                if (std::isfinite(depth) && depth > 0.2f && depth < 5.0f) {
                    float x = (u - cx) * depth / fx;
                    float y = (v - cy) * depth / fy;
                    float z = depth;

                    Eigen::Vector3f pt_cam(x, y, z);
                    Eigen::Vector3f pt_world = R * pt_cam + t;

                    octree_->updateNode(octomap::point3d(pt_world.x(), pt_world.y(), pt_world.z()), true);
                }
            }
        }

        octree_->updateInnerOccupancy();
    }

    // Publish the OctoMap as PointCloud2
    void publish_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);

        octomap::point3d_list occupied_points;
        for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied_points.push_back(it.getCoordinate());
            }
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map";

        cloud_msg.height = 1;
        cloud_msg.width = occupied_points.size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(occupied_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& pt : occupied_points) {
            *iter_x = pt.x(); ++iter_x;
            *iter_y = pt.y(); ++iter_y;
            *iter_z = pt.z(); ++iter_z;
        }

        octomap_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published dense OctoMap with %zu occupied nodes.", occupied_points.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DenseOctoMapMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
