#pragma once


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/core/mat.hpp>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

#include "EKF-SLAM-Example/structs/LandmarkMeasurement.h"

#include "visualization/BackendRos2.h"
#include "visualization/MarkersFactory.h"

#include "visualization/Topics.h"
#include "visualization/Frames.h"
#include "visualization/Colors.h"


class VisualizationEngine {

public:

    VisualizationEngine(std::shared_ptr<rclcpp::Node> node);

    void draw_robot(const rtl::RigidTf3f& pose);
    void draw_landmarks(const std::vector<rtl::Vector3f>& landmarks);
    void draw_landmark_measurements(const std::vector<LandmarkMeasurement>& measurements);
    void draw_covariances();
    void draw_covariance_matrix(const cv::Mat& mat);

    void draw_pose_with_covariance(const rtl::Vector3f& pose, float yaw, const rtl::Matrix33f& cov);

    void draw_estimated_robot(const rtl::RigidTf3f& pose);
    void draw_estimated_landmarks(const std::vector<rtl::Vector3f>& landmarks);

private:

    std::shared_ptr<rclcpp::Node> node_;
    BackendRos2 backend_;
};