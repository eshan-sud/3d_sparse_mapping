#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>

class SparseOctoMapNode : public rclcpp::Node {
public:
    SparseOctoMapNode() : Node("sparse_octomap_node") {
        // Subscribe to ORB-SLAM3 sparse trajectory
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/orb_slam3/trajectory", 10,
            std::bind(&SparseOctoMapNode::path_callback, this, std::placeholders::_1)
        );

        // OctoMap resolution: 10cm
        octree_ = std::make_shared<octomap::OcTree>(0.1);  // Sparse = lower resolution

        // Publisher for visualization in RViz
        octomap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sparse_octomap", 10);

        // Save map every 30 seconds
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&SparseOctoMapNode::save_octomap, this)
        );

        // Publish map every second
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SparseOctoMapNode::publish_octomap, this)
        );

        RCLCPP_INFO(this->get_logger(), "Sparse OctoMap Node initialized.");
        RCLCPP_INFO(this->get_logger(), "For dense map, run 'dense_octomap_node' in parallel.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex map_mutex_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            return;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);
        octree_->clear();

        for (const auto& pose : msg->poses) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;

            octree_->updateNode(octomap::point3d(x, y, z), true);
        }

        octree_->updateInnerOccupancy();
        RCLCPP_INFO(this->get_logger(), "Updated Sparse OctoMap with %zu trajectory points.", msg->poses.size());
    }

    void save_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        std::string save_path = "/home/minor-project/sparse_map.bt";

        if (octree_->writeBinary(save_path)) {
            RCLCPP_INFO(this->get_logger(), "Sparse OctoMap saved to: %s", save_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save Sparse OctoMap.");
        }
    }

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

        for (const auto& point : occupied_points) {
            *iter_x = point.x(); ++iter_x;
            *iter_y = point.y(); ++iter_y;
            *iter_z = point.z(); ++iter_z;
        }

        octomap_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published Sparse OctoMap with %zu occupied nodes.", occupied_points.size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SparseOctoMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
