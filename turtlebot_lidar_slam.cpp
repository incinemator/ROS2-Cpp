#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class TurtleBot3Navigation : public rclcpp::Node {
public:
    TurtleBot3Navigation() : Node("turtlebot3_navigation") {
        // Subscribe to LIDAR scan data
        lidar_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&TurtleBot3Navigation::lidarCallback, this, std::placeholders::_1));

        // Create a publisher for velocity commands
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // SLAM
        // Example: Stop the robot if an obstacle is detected in front
        if (isObstacleInFront(msg)) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);
        } else {
            // Example: Move the robot forward
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;  // Adjust linear velocity as needed
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);
        }
    }

    bool isObstacleInFront(const sensor_msgs::msg::LaserScan::SharedPtr lidar_data) {
        // Check if there is an obstacle in a predefined distance in front
        double obstacle_distance_threshold = 0.5;  // Adjust as needed
        for (double range : lidar_data->ranges) {
            if (range < obstacle_distance_threshold) {
                return true;
            }
        }
        return false;
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleBot3Navigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
