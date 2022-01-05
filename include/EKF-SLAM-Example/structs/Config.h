#pragma once

#include <vector>

#include <rtl/Core.h>
#include <rtl/Transformation.h>

struct EkfConfig {
    float angle_noise = 0.01;
    float distance_noise = 0.1;
    float motion_noise = 0.001;
    float ekf_prediction_period = 0.1;
};

struct SimulationConfig {
    float angle_noise = 0.01;
    float distance_noise = 0.1;
    float motion_noise = 0.001;

    float sensor_range = 1.0;

    float linear_speed_coef = 1.0;
    float angular_speed_coef = 1.0;

    float simulation_loop_period = 0.1;
    float landmark_measurement_period = 1.0f;
};

struct GamepadConfig {
    float gamepad_period = 0.1;
};

struct VisualizationConfig {
    float visualization_period = 0.033;
};

struct Config {
    rtl::RigidTf3f initial_pose{rtl::Rotation3f{0, 0,0},
                                rtl::Translation3f{0, 0, 0}};
    std::vector<rtl::Vector3f> landmarks = {};

    EkfConfig ekf_conf;
    SimulationConfig sim_conf;
    GamepadConfig gamepad_conf;
    VisualizationConfig visualization_config;
};