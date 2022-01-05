#pragma once

#include "Colors.h"
#include <visualization_msgs/msg/marker.hpp>

class MarkersFactory {

public:

    struct Meta {
        rtl::Vector3f pose;
        rtl::Quaternionf orientation;
        rtl::Vector3f scale;
        Colors::Color color;
        std::string text;
        std::vector<rtl::Vector3f> points;
        int id;
        std::string frame;
        rclcpp::Time time;
    };

    static visualization_msgs::msg::Marker create_cylinder(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }

    static visualization_msgs::msg::Marker create_sphere(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }

    static visualization_msgs::msg::Marker create_cube(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }

    static visualization_msgs::msg::Marker create_line_list(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }

    static visualization_msgs::msg::Marker create_line_strip(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }

    static visualization_msgs::msg::Marker create_text(const Meta& meta) {
        auto marker = basic_marker(meta);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        return marker;
    }
private:

    static visualization_msgs::msg::Marker basic_marker(const Meta& meta) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = meta.frame;
        marker.header.stamp = meta.time;
        marker.id = meta.id;
        marker.text = meta.text;
        set_marker_pose(marker, meta);
        set_marker_orientation(marker, meta);
        set_marker_scale(marker, meta);
        set_marker_color(marker, meta);
        set_marker_points(marker, meta);
        return marker;
    }

    static void set_marker_pose(visualization_msgs::msg::Marker& marker, const Meta& meta) {
        marker.pose.position.x = meta.pose.x();
        marker.pose.position.y = meta.pose.y();
        marker.pose.position.z = meta.pose.z();
    }

    static void set_marker_orientation(visualization_msgs::msg::Marker& marker, const Meta& meta) {
        marker.pose.orientation.x = meta.orientation.x();
        marker.pose.orientation.y = meta.orientation.y();
        marker.pose.orientation.z = meta.orientation.z();
        marker.pose.orientation.w = meta.orientation.w();
    }

    static void set_marker_scale(visualization_msgs::msg::Marker& marker, const Meta& meta) {
        marker.scale.x = meta.scale.x();
        marker.scale.y = meta.scale.y();
        marker.scale.z = meta.scale.z();
    }

    static void set_marker_color(visualization_msgs::msg::Marker& marker, const Meta& meta) {
        marker.color.a = meta.color.a;
        marker.color.r = meta.color.r;
        marker.color.g = meta.color.g;
        marker.color.b = meta.color.b;
    }

    static void set_marker_points(visualization_msgs::msg::Marker& marker, const Meta& meta) {
        for(const auto& point : meta.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }
    }

};