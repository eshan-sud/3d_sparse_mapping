#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <memory>

class OctoMapMappingNode : public rclcpp::Node {
public:
    OctoMapMappingNode() : Node("octomap_mapping_node") {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/orb_slam3/trajectory", 10,
            std::bind(&OctoMapMappingNode::path_callback, this, std::placeholders::_1)
        );

        octree_ = std::make_shared<octomap::OcTree>(0.1); // 10cm resolution
        RCLCPP_INFO(this->get_logger(), "OctoMap Mapping Node Initialized.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    std::shared_ptr<octomap::OcTree> octree_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        for (const auto& pose : msg->poses) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;

            // Insert the pose as occupied
            octree_->updateNode(octomap::point3d(x, y, z), true);
        }

        octree_->updateInnerOccupancy();
        RCLCPP_INFO(this->get_logger(), "Inserted %zu trajectory points into OctoMap.", msg->poses.size());

        // Optional: save OctoMap
        // octree_->writeBinary("/tmp/trajectory_map.bt");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
