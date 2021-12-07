#pragma once

#include "Config.h"
#include <rclcpp/rclcpp.hpp>

class SimulationEngine {

public:
    SimulationEngine() = delete;
    SimulationEngine(std::shared_ptr<rclcpp::Node> node, Config conf);

    void set_liner_speed(float speed) {robot_liner_speed_ = speed;};
    void set_angular_speed(float ang_speed) {robot_angular_speed_ = ang_speed;};

protected:

    std::shared_ptr<rclcpp::Node> node_;
    const Config conf_;

    rclcpp::TimerBase::SharedPtr simulation_timer_;

    float robot_liner_speed_;
    float robot_angular_speed_;

    void simulation_step_callback();
};
