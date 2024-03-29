#pragma once

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/core/mat.hpp>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

#include "EKF-SLAM-Example/structs/LandmarkMeasurement.h"
#include "EKF-SLAM-Example/ekf_slam/AgentND.h"

#include "visualization/BackendRos2.h"
#include "visualization/MarkersFactory.h"

#include "visualization/Topics.h"
#include "visualization/Frames.h"
#include "visualization/Colors.h"

#include "EKF-SLAM-Example/ekf_slam/LandmarkND.h"

class VisualizationEngine {

public:

    class Correlation {
    public:
        Correlation() = delete;
        Correlation(const rtl::Vector3f& a, const rtl::Vector3f& b, float c) : pose_a_(a), pose_b_(b), correlation_{c} {}

        [[nodiscard]] const rtl::Vector3f& pose_a() const {return pose_a_;}
        [[nodiscard]] const rtl::Vector3f& pose_b() const {return pose_b_;}
        [[nodiscard]] float correlation() const {return correlation_;}
    private:
        const rtl::Vector3f pose_a_;
        const rtl::Vector3f pose_b_;
        const float correlation_;
    };

    explicit VisualizationEngine(std::shared_ptr<rclcpp::Node> node);

    void draw_robot(const rtl::RigidTf3f& pose);
    void draw_landmarks(const std::vector<rtl::Vector3f>& landmarks);
    void draw_landmark_measurements(const std::vector<LandmarkMeasurement>& measurements);
    void draw_correlations(std::vector<Correlation> correlations);
    void draw_covariance_matrix(const cv::Mat& mat);
    void draw_information_matrix(const cv::Mat& mat);

    void draw_pose_with_covariance(const rtl::Vector3f& pose, float yaw, const rtl::Matrix33f& cov);

    void draw_estimated_robot(const rtl::RigidTf3f& pose);
    void draw_estimated_landmarks(const std::vector<std::pair<rtl::Vector3f, size_t>>& landmarks);
    void draw_measurements_wrt_estimated_robot(const std::vector<LandmarkMeasurement>& measurements, const AgentND<2, float>& agent);

    void draw_real_trajectory(const std::deque<rtl::Vector3f>& trajectory);
    void draw_estimated_trajectory(const std::deque<rtl::Vector3f>& trajectory);
    void draw_landmark_matches(const std::vector<std::pair<LandmarkMeasurement, LandmarkND<2, float>>>& measurements_and_landmarks, const AgentND<2, float>& agent);

private:

    std::shared_ptr<rclcpp::Node> node_;
    BackendRos2 backend_;
};