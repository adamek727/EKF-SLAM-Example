#include "EKF-SLAM-Example/VisualizationEngine.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/highgui.hpp>

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

    auto time_rot = rtl::Quaternionf{0, 0, static_cast<float>(time_diff.seconds())};
    auto landmark_orientation = time_rot;
    for (const auto& landmark : landmarks) {
        MarkersFactory::Meta landmark_meta {
                .pose = landmark,
                .orientation = landmark_orientation,
                .scale = rtl::Vector3f {0.2f, 0.2f, 0.2f},
                .color = Colors::Blue,
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
                .points = {measurement.sensor_pose.trVec() + rtl::Vector3f{0.0f, 0.0f, 0.1f},
                           landmark_pose + rtl::Vector3f{0.0f, 0.0f, 0.1f}},
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

void VisualizationEngine::draw_correlations(std::vector<Correlation> correlations) {

    visualization_msgs::msg::MarkerArray msg;
    int marker_count = 0;
    static int max_marker_count = 0;

    for (const auto& correlation : correlations) {
        float scale = correlation.correlation() * 10;
        MarkersFactory::Meta line_meta {
                .pose = rtl::Vector3f ::zeros(),
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f {scale, scale, scale},
                .color = Colors::Purple,
                .points = {correlation.pose_a(), correlation.pose_b()},
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
    backend_.visualize(msg, topics::CORRELATIONS());
}


void VisualizationEngine::draw_covariance_matrix(const cv::Mat& mat) {

//#ifdef DRAW_OPEN_CV
    cv::Mat upscaled_mat, cutted, norm_f, norm_i;

    double min, max;
    cv::minMaxLoc(mat, &min, &max);
    cutted = mat.clone();
    cutted.setTo(0, cutted >= max);

    normalize(cutted, norm_f, 0, 1, cv::NORM_MINMAX,CV_32FC1);
    cv::resize(norm_f, upscaled_mat, cv::Size(400, 400), cv::INTER_NEAREST);
    cv::imshow("cov mat", upscaled_mat);
    cv::waitKey(1);
//#endif

    auto msg = sensor_msgs::msg::Image();

    std_msgs::msg::Header header;
    header.stamp = node_->get_clock()->now();
    header.frame_id = frames::ORIGIN();

    auto img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, norm_f);
    img_bridge.toImageMsg(msg);

    backend_.visualize(msg, topics::COV_MATRIX());
}


void VisualizationEngine::draw_pose_with_covariance(const rtl::Vector3f& pose, float yaw, const rtl::Matrix33f& cov) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.frame_id = frames::ORIGIN();
    msg.header.stamp = node_->get_clock()->now();

    rtl::Quaternionf q{0.0f, 0.0f, yaw};
    msg.pose.pose.position.x = pose.x();
    msg.pose.pose.position.y = pose.y();
    msg.pose.pose.position.z = 0.01;
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.z = q.z();

    msg.pose.covariance.fill(0);
    msg.pose.covariance.at(0) = cov.getElement(0,0);
    msg.pose.covariance.at(1) = cov.getElement(0,1);
    msg.pose.covariance.at(3) = cov.getElement(0,2);

    msg.pose.covariance.at(6) = cov.getElement(1,0);
    msg.pose.covariance.at(7) = cov.getElement(1,1);
    msg.pose.covariance.at(9) = cov.getElement(1,2);

    msg.pose.covariance.at(18) = cov.getElement(2,0);
    msg.pose.covariance.at(19) = cov.getElement(2,1);
    msg.pose.covariance.at(21) = cov.getElement(2,2);

    backend_.visualize(msg, topics::POSE_WITH_COV());
}


void VisualizationEngine::draw_estimated_robot(const rtl::RigidTf3f& pose) {

    visualization_msgs::msg::MarkerArray msg;
    MarkersFactory::Meta estimated_robot {
            .pose = pose.trVec() + rtl::Vector3f {0.0f, 0.0f, 0.25f},
            .orientation = pose.rotQuaternion(),
            .scale = rtl::Vector3f {0.25f, 0.25f, 0.5f},
            .color = Colors::Aqua,
            .id = 0,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    msg.markers.emplace_back(MarkersFactory::create_cylinder(estimated_robot));
    backend_.visualize(msg, topics::ESTIMATED_ROBOT());
}


void VisualizationEngine::draw_estimated_landmarks(const std::vector<std::pair<rtl::Vector3f, size_t>>& landmarks) {
    static auto time_begin = node_->get_clock()->now();
    static int max_marker_count = 0;

    visualization_msgs::msg::MarkerArray msg;
    int marker_count = 0;

    auto time_diff = node_->get_clock()->now() - time_begin;
    auto landmark_orientation = rtl::Quaternionf{0, 0, static_cast<float>(time_diff.seconds())};

    for (const auto& landmark_id : landmarks) {
        MarkersFactory::Meta landmark_meta {
                .pose = landmark_id.first,
                .orientation = landmark_orientation,
                .scale = rtl::Vector3f {0.2f, 0.2f, 0.4f},
                .color = Colors::Green,
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_sphere(landmark_meta));

        MarkersFactory::Meta landmark_test_meta {
                .pose = landmark_id.first + rtl::Vector3f {0.0f, 0.0f, 0.3f},
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f {0.3f, 0.3f, 0.3f},
                .color = Colors::Green,
                .text = std::to_string(landmark_id.second),
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_text(landmark_test_meta));
    }


    for (;marker_count < max_marker_count;) {
        MarkersFactory::Meta line_meta {
                .pose = rtl::Vector3f ::zeros(),
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f::zeros(),
                .color = Colors::Invisible,
                .id = marker_count++,
                .frame = frames::ORIGIN(),
                .time = node_->get_clock()->now(),
        };
        msg.markers.emplace_back(MarkersFactory::create_sphere(line_meta));
    }
    max_marker_count = marker_count;
    backend_.visualize(msg, topics::ESTIMATED_LANDMARKS());
}


void VisualizationEngine::draw_measurements_wrt_estimated_robot(const std::vector<LandmarkMeasurement>& measurements, const AgentND<2, float>& agent) {

    visualization_msgs::msg::MarkerArray msg;
    int marker_count = 0;
    static int max_marker_count = 0;

    auto agent_pose = rtl::Vector3f{agent.translation().trVecX(), agent.translation().trVecY(), 0.0f};
    auto agent_rotation = rtl::Rotation3f{rtl::Quaternionf{0.0f, 0.0f, agent.rotation().rotAngle()}};

    for (const auto& measurement : measurements) {

        auto relative_landmark_pose = rtl::Vector3f{measurement.range, 0.0f, 0.0f};
        auto relative_landmark_rotation = rtl::Rotation3f{rtl::Quaternionf{0.0f, 0.0f, measurement.yaw}};
        auto landmark_pose = agent_pose +
                             relative_landmark_pose
                                     .transformed(relative_landmark_rotation)
                                     .transformed(agent_rotation);


        MarkersFactory::Meta line_meta {
                .pose = rtl::Vector3f ::zeros(),
                .orientation = rtl::Quaternionf::identity(),
                .scale = rtl::Vector3f {0.02f, 0.02f, 0.02f},
                .color = Colors::White,
                .points = {agent_pose + rtl::Vector3f{0.0f, 0.0f, 0.15f},
                           landmark_pose + rtl::Vector3f{0.0f, 0.0f, 0.15f}},
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
    backend_.visualize(msg, topics::LANDMARK_MEASUREMENTS_WRT_ESTIMATED_ROBOT());
}


void VisualizationEngine::draw_real_trajectory(const std::deque<rtl::Vector3f>& trajectory) {
    static auto time_begin = node_->get_clock()->now();
    int marker_count = 0;

    visualization_msgs::msg::MarkerArray msg;
    MarkersFactory::Meta trajectory_meta {
            .pose = rtl::Vector3f::zeros(),
            .orientation = rtl::Quaternionf ::identity(),
            .scale = rtl::Vector3f {0.1f, 0.0f, 0.0f},
            .color = Colors::Red,
            .id = marker_count++,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    for (const auto& point : trajectory) {
        trajectory_meta.points.push_back(point);
    }
    msg.markers.emplace_back(MarkersFactory::create_line_strip(trajectory_meta));

    backend_.visualize(msg, topics::TRAJECTORY_GT());
}


void VisualizationEngine::draw_estimated_trajectory(const std::deque<rtl::Vector3f>& trajectory) {
    static auto time_begin = node_->get_clock()->now();
    int marker_count = 0;

    visualization_msgs::msg::MarkerArray msg;
    MarkersFactory::Meta trajectory_meta {
            .pose = rtl::Vector3f::zeros(),
            .orientation = rtl::Quaternionf ::identity(),
            .scale = rtl::Vector3f {0.05f, 0.0f, 0.0f},
            .color = Colors::Aqua,
            .id = marker_count++,
            .frame = frames::ORIGIN(),
            .time = node_->get_clock()->now(),
    };
    for (const auto& point : trajectory) {
        trajectory_meta.points.push_back(point + rtl::Vector3f{0.0f, 0.0f, 0.05f});
    }
    msg.markers.emplace_back(MarkersFactory::create_line_strip(trajectory_meta));

    backend_.visualize(msg, topics::ESTIMATED_TRAJECTORY());
}