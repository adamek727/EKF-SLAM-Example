#include "EKF-SLAM-Example/VisualizationEngine.h"

VisualizationEngine::VisualizationEngine(std::shared_ptr<rclcpp::Node> node)
: node_{node}
, backend_{node} {

}

void VisualizationEngine::draw_robot(const rtl::RigidTf3f& pose) {
    visualization_msgs::msg::MarkerArray msg;

    MarkersFactory::Meta robot_body_meta {
            .pose = pose.trVec() + rtl::Vector3f {0.0f, 0.0f, 0.5f},
            .orientation = pose.rotQuaternion(),
            .scale = rtl::Vector3f {0.5f, 0.5f, 1.0f},
            .color = Colors::Red,
            .id = 0,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    msg.markers.emplace_back(MarkersFactory::create_cylinder(robot_body_meta));

    MarkersFactory::Meta robot_head_meta {
            .pose = pose.trVec() + rtl::Vector3f {0.0f, 0.0f, 1.0f},
            .orientation = pose.rotQuaternion(),
            .scale = rtl::Vector3f {0.5f, 0.5f, 0.5f},
            .color = Colors::Red,
            .id = 1,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    msg.markers.emplace_back(MarkersFactory::create_sphere(robot_head_meta));

    rtl::Vector3f relative_left_eye_pose = {0.2f, 0.10f, 1.0f};
    MarkersFactory::Meta robot_eye_left_meta {
            .pose = relative_left_eye_pose.transformed(pose),
            .orientation = pose.rotQuaternion(),
            .scale = rtl::Vector3f {0.1f, 0.1f, 0.1f},
            .color = Colors::Aqua,
            .id = 2,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    msg.markers.emplace_back(MarkersFactory::create_sphere(robot_eye_left_meta));

    rtl::Vector3f relative_right_eye_pose = {0.2f, -0.10f, 1.0f};
    MarkersFactory::Meta robot_eye_right_meta {
            .pose = relative_right_eye_pose.transformed(pose),
            .orientation = pose.rotQuaternion(),
            .scale = rtl::Vector3f {0.1f, 0.1f, 0.1f},
            .color = Colors::Aqua,
            .id = 3,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    msg.markers.emplace_back(MarkersFactory::create_sphere(robot_eye_right_meta));

    backend_.visualize(msg, topics::ROBOT());
}

void VisualizationEngine::draw_landmarks(const std::vector<rtl::Vector3f>& landmarks) {

    static auto time_begin = node_->get_clock()->now();

    visualization_msgs::msg::MarkerArray msg;
    int marker_count = 0;

    auto time_diff = node_->get_clock()->now() - time_begin;


//    auto pitch_rot = rtl::Quaternionf{0, M_PI_4, 0};
//    auto yaw_rot = rtl::Quaternionf{M_PI_4, 0, 0};
    auto time_rot = rtl::Quaternionf{0, 0, static_cast<float>(time_diff.seconds())};
    auto landmark_orientation = time_rot; /* * yaw_rot * pitch_rot * rtl::Quaternionf::identity();*/
    for (const auto& landmark : landmarks) {
        MarkersFactory::Meta landmark_meta {
                .pose = landmark,
                .orientation = landmark_orientation,
                .scale = rtl::Vector3f {0.2f, 0.2f, 0.2f},
                .color = Colors::Green,
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_cube(landmark_meta));
    }
    backend_.visualize(msg, topics::LANDMARKS());
}

void VisualizationEngine::draw_landmark_measurements(const std::vector<LandmarkMeasurement>& measurements) {
    visualization_msgs::msg::MarkerArray msg;
    int marker_count = 0;
    static int max_marker_count = 0;

    for (const auto& measurement : measurements) {

        auto relative_landmark_pose = rtl::Vector3f{measurement.range, 0.0f, 0.0f};
        auto relative_landmark_rotation = rtl::Rotation3f{rtl::Quaternionf{0.0f, 0.0f, measurement.yaw}};
        auto landmark_pose = measurement.sensor_pose.trVec() +
                             relative_landmark_pose
                                     .transformed(relative_landmark_rotation)
                                     .transformed(measurement.sensor_pose.rot());


        MarkersFactory::Meta line_meta {
                .pose = rtl::Vector3f ::zeros(),
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f {0.05f, 0.05f, 0.05f},
                .color = Colors::Yellow,
                .points = {measurement.sensor_pose.trVec(), landmark_pose},
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_line_list(line_meta));
    }
    for (;marker_count < max_marker_count;) {
        MarkersFactory::Meta line_meta {
                .pose = rtl::Vector3f ::zeros(),
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f::zeros(),
                .color = Colors::Invisible,
                .points = {},
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_line_list(line_meta));
    }

    max_marker_count = marker_count;
    backend_.visualize(msg, topics::LANDMARK_MEASUREMENTS());
}

void VisualizationEngine::draw_covariances() {

}

void draw_covariance_matrix() {

}