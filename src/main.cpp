#include <iostream>
#include <rclcpp/rclcpp.hpp>


#include "YAML-Service/YAML-Service.h"
#include "EKF-SLAM-Example/Controller.h"
#include "EKF-SLAM-Example/structs/Config.h"

int main(int argc, char* argv[]) {

    if (argc < 2) {
        std::cout << "To few arguments!" << std::endl;
        std::cout << "Usage: ekf-slam-example <path-to-config-file>" << std::endl;
        return 1;
    }

    Config conf;
    YamlService yaml_config(argv[1]);

    try {

        // Reading pose
        auto init_pose = yaml_config.getFloatArray({"init_pose", "pose"});
        auto init_orientation = yaml_config.getFloatArray({"init_pose", "orientation"});
        auto translation = rtl::Translation3f {init_pose[0], init_pose[1], init_pose[2]};
        auto orientation = rtl::Rotation3f(init_orientation[0], init_orientation[1], init_orientation[2]);
        conf.initial_pose = rtl::RigidTf3f {orientation, translation};

        // Reasing Landmarks
        for (const auto& node : yaml_config.getNodeArray({"landmarks"})) {
            const auto landmark_pose = node.as<std::vector<float>>();
            conf.landmarks.emplace_back(rtl::Vector3f{landmark_pose[0], landmark_pose[1], landmark_pose[2]});
        }


        conf.ekf_conf = EkfConfig{
            .angle_noise = yaml_config.getFloatValue({"ekf_slam", "angle_noise"}),
            .distance_noise = yaml_config.getFloatValue({"ekf_slam", "distance_noise"}),
            .motion_noise = yaml_config.getFloatValue({"ekf_slam", "motion_noise"}),
            .ekf_prediction_period = yaml_config.getFloatValue({"ekf_slam", "ekf_prediction_period"}),
        };

        conf.sim_conf = SimulationConfig{
            .angle_noise = yaml_config.getFloatValue({"simulation", "angle_noise"}),
            .distance_noise = yaml_config.getFloatValue({"simulation", "distance_noise"}),
            .motion_noise = yaml_config.getFloatValue({"simulation", "motion_noise"}),
            .sensor_range = yaml_config.getFloatValue({"simulation", "sensor_range"}),
            .linear_speed_coef = yaml_config.getFloatValue({"simulation", "linear_speed_coef"}),
            .angular_speed_coef = yaml_config.getFloatValue({"simulation", "angular_speed_coef"}),
            .simulation_loop_period = yaml_config.getFloatValue({"simulation", "loop_period"}),
            .landmark_measurement_period = yaml_config.getFloatValue({"simulation", "landmark_measurement_period"}),
        };

        conf.gamepad_conf = GamepadConfig {
            .gamepad_period = yaml_config.getFloatValue({"gamepad", "gamepad_period"}),
        };

        conf.visualization_config = VisualizationConfig {
            .visualization_period = yaml_config.getFloatValue({"visualization", "visualization_period"}),
        };

    } catch (std::exception& e) {
        std::cerr << "Exception when reading config: " << e.what() << std::endl;
        return 2;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>(conf));
    rclcpp::shutdown();
    return 0;
}
