#pragma once

#include <rclcpp/rclcpp.hpp>
#include "structs/Config.h"
#include "SimulationEngine.h"
#include "GamepadHandler.h"
#include "VisualizationEngine.h"

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

    void visualization_timer_callback();
};
