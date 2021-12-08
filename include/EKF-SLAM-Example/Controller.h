#pragma once

#include <rclcpp/rclcpp.hpp>
#include "structs/Config.h"
#include "SimulationEngine.h"
#include "GamepadHandler.h"

class Controller : public rclcpp::Node {

public:

    Controller() = delete;
    Controller(const Config& conf);

protected:

    const Config conf_;
    SimulationEngine simulation_engine_;
    GamepadHandler gamepad_handler_;
};
