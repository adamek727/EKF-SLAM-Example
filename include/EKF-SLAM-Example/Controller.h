#pragma once

#include <rclcpp/rclcpp.hpp>
#include "structs/Config.h"
#include "SimulationEngine.h"
#include "GamepadHandler.h"

#define DRAW_OPEN_CV
#include "VisualizationEngine.h"
#include "ekf_slam/EkfSlam2D.h"

class Controller : public rclcpp::Node {

    static constexpr size_t num_of_landmarks = 15;
    static constexpr float landmark_assing_distance = 0.5f;

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

    void ekf_prediction_timer_callback();
    void visualization_timer_callback();
    void visualize_covariance();
};
