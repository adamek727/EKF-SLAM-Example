#include "EKF-SLAM-Example/Controller.h"

Controller::Controller(const Config& conf)
        : Node("ekf_slam_example")
        , conf_{conf}
        , simulation_engine_(static_cast<std::shared_ptr<Node>>(this), conf)
        , gamepad_handler_{static_cast<std::shared_ptr<Node>>(this)}
        , visualization_engine_{static_cast<std::shared_ptr<Node>>(this)} {

    gamepad_handler_.set_joystick_event_callback([&](){
        simulation_engine_.set_liner_speed(gamepad_handler_.get_right_x());
        simulation_engine_.set_angular_speed(gamepad_handler_.get_left_y());
    });

    simulation_engine_.set_landmark_callback([&](){
        auto landmarks = simulation_engine_.get_landmark_measurements();
        ekf_slam_.correct({});
    });

    auto ekf_prediction_period_ms = static_cast<size_t>(conf.ekf_prediction_period * 1000.0f);
    visualization_timer_ = create_wall_timer(std::chrono::milliseconds(ekf_prediction_period_ms),
                                             std::bind(&Controller::ekf_prediction_timer_callback,
                                             this));

    auto visualization_period_ms = static_cast<size_t>(conf.visualization_period * 1000.0f);
    visualization_timer_ = create_wall_timer(std::chrono::milliseconds(visualization_period_ms),
                                             std::bind(&Controller::visualization_timer_callback,
                                             this));
}


void Controller::ekf_prediction_timer_callback() {
    auto linear_speed = 0.0f;
    auto angular_speed = 0.0f;
    ekf_slam_.predict(linear_speed, angular_speed, conf_.ekf_prediction_period);
}


void Controller::visualization_timer_callback() {
    visualization_engine_.draw_robot(simulation_engine_.get_robot_pose());
    visualization_engine_.draw_landmarks(conf_.landmarks);
    visualization_engine_.draw_landmark_measurements(simulation_engine_.get_landmark_measurements());
}