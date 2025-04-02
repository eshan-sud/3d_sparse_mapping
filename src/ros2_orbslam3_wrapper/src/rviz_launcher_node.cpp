#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cstdlib>   // For system() command
#include <csignal>   // For kill()
#include <unistd.h>  // For fork(), getpid(), execlp(), setsid()
#include <sys/types.h>
#include <sys/wait.h>

class RVizLauncher : public rclcpp::Node {
public:
    RVizLauncher() : Node("rviz_launcher"), rviz_pid_(-1) {
        RCLCPP_INFO(this->get_logger(), "Launching RViz...");

        // Fork the process to launch RViz
        rviz_pid_ = fork();
        if (rviz_pid_ == 0) {
            // Child process: Create new session and execute rviz2
            if (setsid() == -1) {
                perror("setsid failed");
                _exit(EXIT_FAILURE);
            }
            execlp("rviz2", "rviz2", (char *)NULL);
            // If execlp fails, log error and exit
            perror("Failed to start RViz");
            _exit(EXIT_FAILURE);
        } else if (rviz_pid_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork RViz process.");
            throw std::runtime_error("Failed to fork RViz process.");
        }

        // Set up an image subscriber
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            [this](sensor_msgs::msg::Image::UniquePtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received image (not processing yet)");
            }
        );

        RCLCPP_INFO(this->get_logger(), "RViz launcher node initialized.");
    }

    ~RVizLauncher() {
        RCLCPP_INFO(this->get_logger(), "Shutting down RViz...");
        if (rviz_pid_ > 0) {
            kill(-rviz_pid_, SIGTERM);  // Kill all processes in the session
            waitpid(rviz_pid_, NULL, 0);  // Reap the process
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    pid_t rviz_pid_;  // Store the PID of RViz
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RVizLauncher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
