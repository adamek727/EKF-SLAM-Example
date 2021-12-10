#pragma once

#include <rclcpp/rclcpp.hpp>
#include <random>

#include "structs/Config.h"
#include "structs/LandmarkMeasurement.h"

class SimulationEngine {

public:
    SimulationEngine() = delete;
    SimulationEngine(std::shared_ptr<rclcpp::Node> node, const Config conf);

    void set_liner_speed(float speed) {robot_liner_speed_ = speed * conf_.linear_speed_coef;};
    void set_angular_speed(float ang_speed) {robot_angular_speed_ = ang_speed * conf_.angular_speed_coef;};

    void set_landmark_callback(std::function<void()> f) {landmark_measured_callback_ = f;}

    [[nodiscard]] const rtl::RigidTf3f & get_robot_pose() const {return robot_pose_;}

    [[nodiscard]] const std::vector<LandmarkMeasurement>& get_landmark_measurements() const {return landmark_measurements_;};

protected:

    std::shared_ptr<rclcpp::Node> node_;
    const Config conf_;

    rclcpp::TimerBase::SharedPtr simulation_timer_;
    rclcpp::TimerBase::SharedPtr landmarks_measurement_timer_;

    std::function<void()> landmark_measured_callback_ = {};
    std::vector<LandmarkMeasurement> landmark_measurements_;

    rtl::RigidTf3f robot_pose_;
    float robot_liner_speed_;
    float robot_angular_speed_;

    void simulation_step_timer_callback();
    void landmark_measurement_timer_callback();
    static float gaussian_random_val(float mean, float std_div);
};
