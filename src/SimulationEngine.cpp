#include "EKF-SLAM-Example/SimulationEngine.h"

SimulationEngine::SimulationEngine(std::shared_ptr<rclcpp::Node> node, const Config& conf)
        : node_{std::move(node)}
        , conf_{std::move(conf)} {

    robot_pose_ = conf.initial_pose;

    auto simulation_period_ms = static_cast<size_t>(conf.sim_conf.simulation_loop_period * 1000.0f);
    simulation_timer_ = node_->create_wall_timer(std::chrono::milliseconds(simulation_period_ms),
                                                 std::bind(&SimulationEngine::simulation_step_timer_callback,
                                                 this));

    auto landmark_measurement_period_ms = static_cast<size_t>(conf.sim_conf.landmark_measurement_period * 1000.0f);
    landmarks_measurement_timer_ = node_->create_wall_timer(std::chrono::milliseconds(landmark_measurement_period_ms),
                                                            std::bind(&SimulationEngine::landmark_measurement_timer_callback,
                                                            this));
}


void SimulationEngine::simulation_step_timer_callback() {

    // rotate
    rtl::Rotation3f rot(0.0f, 0.0f, robot_angular_speed_ * conf_.sim_conf.simulation_loop_period);
    float r, p, y;
    robot_pose_.rotRpy(r, p, y);
    robot_pose_ = rtl::RigidTf3f{rtl::Rotation3f{r, p, y + robot_angular_speed_ * conf_.sim_conf.simulation_loop_period},
                                 robot_pose_.tr()};

    // translate
    rtl::Vector3f tr_vector{robot_liner_speed_ * conf_.sim_conf.simulation_loop_period, 0.0f, 0.0f};
    auto noise = rtl::Vector3f{gaussian_random_val(0.0, conf_.sim_conf.motion_noise),
                               gaussian_random_val(0.0, conf_.sim_conf.motion_noise),
                               0.0f};

    tr_vector.transform(robot_pose_.rot());
    tr_vector += noise * conf_.sim_conf.simulation_loop_period;
    rtl::Translation3f trans{tr_vector};

    robot_pose_.transform(trans);
}


void SimulationEngine::landmark_measurement_timer_callback() {
    landmark_measurements_.clear();
    for (const auto& landmark : conf_.landmarks) {
        auto distance = (landmark - robot_pose_.trVec()).length();
        if (distance <= conf_.sim_conf.sensor_range) {

            auto robot_landmark_pose_diff = rtl::Vector3f{landmark.x(), landmark.y(), 0.0f} - rtl::Vector3f{robot_pose_.trVecX(), robot_pose_.trVecY(), 0.0f};
            rtl::Rotation3f measurement_orientation(rtl::Vector3f{1.0f, 0.0f, 0.0f}, robot_landmark_pose_diff);
            measurement_orientation.transform(robot_pose_.rot().inverted());
            float r,p,y;
            measurement_orientation.rotRpy(r, p, y);

            landmark_measurements_.emplace_back(
                    LandmarkMeasurement{
                        .sensor_pose = robot_pose_,
                        .pitch = 0.0f,
                        .yaw = y + gaussian_random_val(0.0, conf_.sim_conf.angle_noise),
                        .range = distance + gaussian_random_val(0.0, conf_.sim_conf.distance_noise)});
        }
    }
    std::cout << "no of measurements: " << landmark_measurements_.size()<< std::endl;
    landmark_measured_callback_();
    landmark_measurements_.clear();
}

float SimulationEngine::gaussian_random_val(float mean, float std_div) {
    static std::random_device r;
    static auto engine = std::default_random_engine(r());
    std::normal_distribution<float> distribution(mean, std_div);
    return distribution(engine);
}