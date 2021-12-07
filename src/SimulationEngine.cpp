#include "EKF-SLAM-Example/SimulationEngine.h"

SimulationEngine::SimulationEngine(std::shared_ptr<rclcpp::Node> node, Config conf)
        : node_{std::move(node)}
        , conf_{std::move(conf)} {

    auto simulation_periond_ms = static_cast<size_t>(conf.simulation_loop_period*1000.0f);
    simulation_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(simulation_periond_ms),
            std::bind(&SimulationEngine::simulation_step_callback, this));
}


void SimulationEngine::simulation_step_callback() {
    std::cout << robot_liner_speed_ << " " << robot_angular_speed_ << std::endl;
}