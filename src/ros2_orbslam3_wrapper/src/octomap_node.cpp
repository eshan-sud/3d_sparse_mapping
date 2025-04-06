#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <string>
#include <chrono>

class OctoMapMappingNode : public rclcpp::Node {
public:
    // Constructor
    OctoMapMappingNode() : Node("octomap_mapping_node") {
        // Subscribe to ORB-SLAM3 trajectory
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/orb_slam3/trajectory", 10,
            std::bind(&OctoMapMappingNode::path_callback, this, std::placeholders::_1)
        );

        // Timer to save the map every 30 seconds
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&OctoMapMappingNode::save_octomap, this)
        );

        // Create OctoMap with resolution = 10cm
        octree_ = std::make_shared<octomap::OcTree>(0.1);  // Resolution 0.1 meters

        // Publisher to visualize in RViz (as a raw binary message)
        octomap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap_full", 10);

        // Periodic timer to publish the OctoMap
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OctoMapMappingNode::publish_octomap, this)
        );

        RCLCPP_INFO(this->get_logger(), "OctoMap Mapping Node Initialized.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex map_mutex_;

    // Callback function to handle the incoming trajectory data
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            return;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);

        // Process each trajectory point and update the Octree
        for (const auto& pose : msg->poses) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;

            // Mark the point as occupied in the Octree
            octree_->updateNode(octomap::point3d(x, y, z), true); // Mark as occupied
        }

        octree_->updateInnerOccupancy();  // Update inner nodes
        RCLCPP_INFO(this->get_logger(), "Updated OctoMap with %zu trajectory points.", msg->poses.size());
    }

    // Save the current OctoMap to a file
    void save_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        std::string save_path = "/home/minor-project/map.bt";  // Save path
        if (octree_->writeBinary(save_path)) {
            RCLCPP_INFO(this->get_logger(), "OctoMap saved to: %s", save_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save OctoMap.");
        }
    }

    // Publish the current OctoMap to a ROS topic
    void publish_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);

        // Convert the Octree to PointCloud2 (for visualization in RViz)
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "map"; // Or "world" depending on your setup

        // Get all occupied nodes from the Octree
        octomap::point3d_list occupied_points;
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if (octree_->isNodeOccupied(*it)) {
                occupied_points.push_back(it.getCoordinate());  // Add the occupied points
            }
        }

        // Resize the cloud message and add points
        cloud_msg.height = 1;  // Single row of points
        cloud_msg.width = occupied_points.size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        // Define fields for the PointCloud2 message
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");  // Fields to store x, y, z
        modifier.resize(occupied_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        // Add the points to the cloud message
        for (const auto& point : occupied_points) {
            *iter_x = point.x(); ++iter_x;
            *iter_y = point.y(); ++iter_y;
            *iter_z = point.z(); ++iter_z;
        }

        // Publish the OctoMap as a PointCloud2 message
        octomap_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published OctoMap with %zu occupied nodes.", occupied_points.size());
    }
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<OctoMapMappingNode>();  // Create the node
    rclcpp::spin(node);  // Spin to keep the node running
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}
