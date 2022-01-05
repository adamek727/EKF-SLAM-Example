#pragma once

#include <deque>

#include <rclcpp/rclcpp.hpp>

#include <rtl/Algorithms.h>

#include "structs/Config.h"
#include "SimulationEngine.h"
#include "GamepadHandler.h"

#define DRAW_OPEN_CV
#include "VisualizationEngine.h"
#include "ekf_slam/EkfSlam2D.h"

class Controller : public rclcpp::Node {

    static constexpr size_t num_of_landmarks = 20;
    static constexpr float landmark_assing_distance = 1.0f;
    static constexpr size_t trajectory_history_size = 500;

public:

    Controller() = delete;
    Controller(const Config& conf);
    ~Controller();

protected:

    const Config conf_;
    SimulationEngine simulation_engine_;
    GamepadHandler gamepad_handler_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::TimerBase::SharedPtr ekf_prediction_timer_;
    VisualizationEngine visualization_engine_;
    EkfSlam2D<float, num_of_landmarks> ekf_slam_;

    std::deque<rtl::Vector3f> trajectory_gt_;
    std::deque<rtl::Vector3f> trajectory_estimated_;

    void ekf_prediction_timer_callback();
    void visualization_timer_callback();
    void visualize_covariance();

    std::array<rtl::Munkres<float, num_of_landmarks>::Result, num_of_landmarks> assign_measurements_to_landmarks(const std::vector<LandmarkMeasurement>& measurements);
};
