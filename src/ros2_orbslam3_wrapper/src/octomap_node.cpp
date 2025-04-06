#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <memory>
#include <string>
#include <chrono>

class OctoMapMappingNode : public rclcpp::Node {
public:
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
        octree_ = std::make_shared<octomap::OcTree>(0.1);

        RCLCPP_INFO(this->get_logger(), "OctoMap Mapping Node Initialized.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::mutex map_mutex_;  // Protect OctoMap access

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            return;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto& pose : msg->poses) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;

            octree_->updateNode(octomap::point3d(x, y, z), true); // Mark as occupied
        }

        octree_->updateInnerOccupancy();
        RCLCPP_INFO(this->get_logger(), "Updated OctoMap with %zu trajectory points.", msg->poses.size());
    }

    void save_octomap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        std::string save_path = "/home/minor-project/map.bt";
        if (octree_->writeBinary(save_path)) {
            RCLCPP_INFO(this->get_logger(), "OctoMap saved to: %s", save_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save OctoMap.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
