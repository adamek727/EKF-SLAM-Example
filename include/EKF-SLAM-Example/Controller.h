#pragma once

#include <rclcpp/rclcpp.hpp>
#include "structs/Config.h"
#include "SimulationEngine.h"
#include "GamepadHandler.h"
#include "VisualizationEngine.h"
#include "ekf_slam/EkfSlam2D.h"

class Controller : public rclcpp::Node {

public:

    Controller() = delete;
    Controller(const Config& conf);

protected:

    const Config conf_;
    SimulationEngine simulation_engine_;
    GamepadHandler gamepad_handler_;

    rclcpp::TimerBase::SharedPtr visualization_timer_;
    VisualizationEngine visualization_engine_;
    EkfSlam2D<float, 50> ekf_slam_;

    void ekf_prediction_timer_callback();
    void visualization_timer_callback();
};
