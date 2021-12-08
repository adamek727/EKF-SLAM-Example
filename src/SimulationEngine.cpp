#include "EKF-SLAM-Example/SimulationEngine.h"

SimulationEngine::SimulationEngine(std::shared_ptr<rclcpp::Node> node, const Config conf)
        : node_{std::move(node)}
        , conf_{std::move(conf)} {

    robot_pose_ = conf.initial_pose;

    auto simulation_period_ms = static_cast<size_t>(conf.simulation_loop_period * 1000.0f);
    simulation_timer_ = node_->create_wall_timer(std::chrono::milliseconds(simulation_period_ms),
                                                 std::bind(&SimulationEngine::simulation_step_timer_callback,
                                                 this));

    auto landmark_measurement_period_ms = static_cast<size_t>(conf.landmark_measurement_period * 1000.0f);
    landmarks_measurement_timer_ = node_->create_wall_timer(std::chrono::milliseconds(landmark_measurement_period_ms),
                                                            std::bind(&SimulationEngine::landmark_measurement_timer_callback,
                                                            this));
}


void SimulationEngine::simulation_step_timer_callback() {

    rtl::Rotation3f rot(0.0f, 0.0f, robot_angular_speed_ * conf_.simulation_loop_period);
    float r, p, y;
    robot_pose_.rotRpy(r, p, y);
    robot_pose_ = rtl::RigidTf3f{rtl::Rotation3f{r, p, y + robot_angular_speed_ * conf_.simulation_loop_period},
                                 robot_pose_.tr()};


    rtl::Vector3f tr_vector{robot_liner_speed_ * conf_.simulation_loop_period, 0.0f, 0.0f};
    tr_vector.transform(robot_pose_.rot());
    rtl::Translation3f trans{tr_vector};
    robot_pose_.transform(trans);
}


void SimulationEngine::landmark_measurement_timer_callback() {
    landmark_measurements_.clear();
    for (const auto& landmark : conf_.landmarks) {
        auto distance = (landmark - robot_pose_.trVec()).length();
        if (distance <= conf_.sensor_range) {

            auto robot_landmark_pose_diff = rtl::Vector3f{landmark.x(), landmark.y(), 0.0f} - rtl::Vector3f{robot_pose_.trVecX(), robot_pose_.trVecY(), 0.0f};
            rtl::Rotation3f measurement_orientation(rtl::Vector3f{1.0f, 0.0f, 0.0f}, robot_landmark_pose_diff);

            float r,p,y;
            measurement_orientation.rotRpy(r, p, y);

            landmark_measurements_.emplace_back(
                    LandmarkMeasurement{
                        .pitch = 0.0f,
                        .yaw = y + conf_.angle_noise,
                        .range = distance + conf_.distance_noise});
        }
    }
    landmark_measured_callback_();
}