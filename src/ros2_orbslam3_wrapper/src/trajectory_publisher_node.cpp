#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        this->declare_parameter<std::string>("trajectory_file", "");
        std::string trajectory_file;
        this->get_parameter("trajectory_file", trajectory_file);

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("orbslam3/trajectory", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&TrajectoryPublisher::publish_trajectory, this));

        path_.header.frame_id = "map";
        load_trajectory(trajectory_file);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_;

    void load_trajectory(const std::string &file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
                RCLCPP_WARN(this->get_logger(), "Malformed line: %s", line.c_str());
                continue;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
            pose.header.frame_id = "map";
            pose.pose.position.x = tx;
            pose.pose.position.y = ty;
            pose.pose.position.z = tz;
            pose.pose.orientation.x = qx;
            pose.pose.orientation.y = qy;
            pose.pose.orientation.z = qz;
            pose.pose.orientation.w = qw;

            path_.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory poses.", path_.poses.size());
    }

    void publish_trajectory() {
        path_.header.stamp = this->now();
        publisher_->publish(path_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
