#pragma once

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

private:

    std::shared_ptr<rclcpp::Node> node_;
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_publishers_;
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_array_publishers_;
};