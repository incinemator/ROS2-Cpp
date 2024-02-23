#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VisualSLAMNode : public rclcpp::Node {
public:
    VisualSLAMNode() : Node("visual_slam_node") {
        // Subscribe to the RealSense D415 camera topic
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&VisualSLAMNode::imageCallback, this, std::placeholders::_1));

        // Initialize Visual SLAM components
        feature_detector_ = cv::ORB::create();
        descriptor_extractor_ = cv::ORB::create();

        // Start the Visual SLAM processing loop
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VisualSLAMNode::processVisualSLAM, this));

        // Initialize data structures
        prev_keypoints_.clear();
        prev_descriptors_ = cv::Mat();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image message to OpenCV format
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Process the image and update Visual SLAM data
            cv::Ptr<cv::Feature2D> feature_detector_;
            cv::Ptr<cv::Feature2D> descriptor_extractor_;

            std::vector<cv::KeyPoint> prev_keypoints_;
            cv::Mat prev_descriptors_;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV_Bridge exception: %s", e.what());
        }
    }

    void processVisualSLAM() {
            if (prev_keypoints_.empty() || prev_descriptors_.empty()) {
                // Initial frame, just store keypoints and descriptors
                prev_keypoints_ = current_keypoints_;
                prev_descriptors_ = current_descriptors_;
                return;
            }

            // Feature matching
            std::vector<cv::DMatch> matches;
            feature_matcher_.match(prev_descriptors_, current_descriptors_, matches);

            // Filter out good matches based on Lowe's ratio test
            double ratio_thresh = 0.7;
            std::vector<cv::DMatch> good_matches;
            for (const auto& match : matches) {
                if (match.distance < ratio_thresh * match.trainIdx) {
                    good_matches.push_back(match);
                }
            }

            // Estimate the homography matrix using RANSAC
            std::vector<cv::Point2f> prev_points, current_points;
            for (const auto& match : good_matches) {
                prev_points.push_back(prev_keypoints_[match.queryIdx].pt);
                current_points.push_back(current_keypoints_[match.trainIdx].pt);
            }

            cv::Mat homography = cv::findHomography(prev_points, current_points, cv::RANSAC);

            // Apply the homography to get the relative transformation
            cv::Mat translation, rotation;
            cv::decomposeHomographyMat(homography, cv::Mat::eye(3, 3, CV_64F), translation, rotation, cv::noArray());

            // Update the Visual SLAM state or perform further processing as needed

            // Publish the estimated pose
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = now();
            pose_msg.pose.pose.position.x = translation.at<double>(0);
            pose_msg.pose.pose.position.y = translation.at<double>(1);
            pose_msg.pose.pose.position.z = translation.at<double>(2);
            // Convert rotation matrix to quaternion
            cv::Mat rotation_mat;
            cv::Rodrigues(rotation, rotation_mat);
            cv::Mat quaternion;
            cv::Rodrigues(rotation_mat, quaternion);
            pose_msg.pose.pose.orientation.x = quaternion.at<double>(0);
            pose_msg.pose.pose.orientation.y = quaternion.at<double>(1);
            pose_msg.pose.pose.orientation.z = quaternion.at<double>(2);
            pose_msg.pose.pose.orientation.w = quaternion.at<double>(3);
            pose_publisher_->publish(pose_msg);

            // Update the previous keypoints and descriptors
            prev_keypoints_ = current_keypoints_;
            prev_descriptors_ = current_descriptors_;
            }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualSLAMNode>());
    rclcpp::shutdown();
    return 0;
}
