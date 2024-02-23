#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class JackalNavigationNode : public rclcpp::Node {
public:
    JackalNavigationNode() : Node("jackal_navigation_node") {
        // Initialize subscribers, publishers, and other components here

        // Example: Subscribe to odometry
        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10, std::bind(&JackalNavigationNode::odomCallback, this, std::placeholders::_1));

        // Example: Subscribe to point cloud from RealSense stereo camera
        pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "realsense/points", 10, std::bind(&JackalNavigationNode::pointcloudCallback, this, std::placeholders::_1));

        // Example: Create a publisher for velocity commands
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Example: Initialize the transform listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    }

private:
    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Process odometry data
        // Process odometry data
        auto linear_velocity_x = msg->twist.twist.linear.x;
        auto angular_velocity_z = msg->twist.twist.angular.z;

        // Example: Compute velocity commands
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = computeLinearVelocity(linear_velocity_x);
        cmd_vel.angular.z = computeAngularVelocity(angular_velocity_z);

        // Publish velocity commands
        cmd_vel_publisher_->publish(cmd_vel);

        // Navigation
        const double distance_threshold = 5.0;  // Adjust as needed

        if (distanceTraveled(msg->pose.pose.position) < distance_threshold) {
            // Continue moving forward
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.2;  // Adjust linear velocity as needed
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);
        } else {
            // Stop the robot
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);

            // Example: Trigger the next navigation action (e.g., set a new goal)
            navigateToNextGoal();
    }

    double distanceTraveled(const geometry_msgs::msg::Point& current_position) {
        // Compute Euclidean distance from the initial position
        double dx = current_position.x - initial_position_.x;
        double dy = current_position.y - initial_position_.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Process point cloud data
        // Implement your perception or mapping logic here
    }

    // Other member functions for navigation logic

    // Members
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JackalNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
