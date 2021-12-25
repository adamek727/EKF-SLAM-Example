#pragma once

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/image.hpp>

class BackendRos2 {

public:

    BackendRos2(std::shared_ptr<rclcpp::Node> node)
            : node_{node} {

    }

    void visualize(const visualization_msgs::msg::Marker& marker, const std::string& topic) {
        if (marker_publishers_.count(topic) == 0) {
            marker_publishers_[topic] = node_->create_publisher<visualization_msgs::msg::Marker>(topic, 0);
        }
        marker_publishers_[topic]->publish(marker);
    }

    void visualize(const visualization_msgs::msg::MarkerArray& markerArray, const std::string& topic) {
        if (marker_array_publishers_.count(topic) == 0) {
            marker_array_publishers_[topic] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 0);
        }
        marker_array_publishers_[topic]->publish(markerArray);
    }

    void visualize(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_with_cov, const std::string& topic) {
        if (pose_with_cov_stamp_publishers_.count(topic) == 0) {
            pose_with_cov_stamp_publishers_[topic] = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, 0);
        }
        pose_with_cov_stamp_publishers_[topic]->publish(pose_with_cov);
    }

    void visualize(const sensor_msgs::msg::Image& image, const std::string& topic) {
        if (image_publishers_.count(topic) == 0) {
            image_publishers_[topic] = node_->create_publisher<sensor_msgs::msg::Image>(topic, 0);
        }
        image_publishers_[topic]->publish(image);
    }

private:

    std::shared_ptr<rclcpp::Node> node_;
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_publishers_;
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_array_publishers_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_with_cov_stamp_publishers_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
};