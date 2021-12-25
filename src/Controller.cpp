#include "EKF-SLAM-Example/Controller.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <rtl/Algorithms.h>

Controller::Controller(const Config& conf)
        : Node("ekf_slam_example")
        , conf_{conf}
        , simulation_engine_(static_cast<std::shared_ptr<Node>>(this), conf)
        , gamepad_handler_{static_cast<std::shared_ptr<Node>>(this), conf}
        , visualization_engine_{static_cast<std::shared_ptr<Node>>(this)}
        , ekf_slam_{0.0001, 0.1} {

    gamepad_handler_.set_joystick_event_callback([&](){
        simulation_engine_.set_liner_speed(gamepad_handler_.get_right_x());
        simulation_engine_.set_angular_speed(gamepad_handler_.get_left_y());
    });

    simulation_engine_.set_landmark_callback([&](){
        auto landmark_measurements = simulation_engine_.get_landmark_measurements();
        auto slam_landmarks = ekf_slam_.get_landmarks();
        auto robot_pose = simulation_engine_.get_robot_pose();

        std::cout << " - - - - - - " << std::endl;
        for (const auto& m : landmark_measurements) {
            auto pose = m.to_xy();
            std::cout << "x: " << pose.x() << " y:" << pose.y() << std::endl;
        }

        // landmark assignment
        std::vector<LandmarkND<2, float>> measurements;
        auto cost = rtl::Matrix<num_of_landmarks, num_of_landmarks, float>::zeros();
        for (size_t m = 0 ; m < landmark_measurements.size() ; m+=1) {
            const auto& measurement = landmark_measurements.at(m);
            const auto m_pose = measurement.to_xy();

            for (size_t s = 0 ; s < slam_landmarks.size() ; s+=1) {
                const auto slam_landmark = slam_landmarks.at(s);
                auto dist = sqrt(pow(slam_landmark.translation().trVecX() - m_pose.x(), 2.0f) +
                                    pow(slam_landmark.translation().trVecY() - m_pose.y(), 2.0f));
                auto c = landmark_assing_distance - std::max(std::min(dist, 0.5f), 0.0f);
                cost.setElement(m, s, c);
            }
        }
        auto assignment_results = rtl::Munkres<float, num_of_landmarks>::solve(cost, true);

        for (size_t m = 0 ; m < landmark_measurements.size() ; m+=1) {
            const auto result = assignment_results.at(m);
            auto pose = landmark_measurements.at(result.col).to_xy();
            if (result.cost > 0.0f) {
                measurements.push_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{pose.x(), pose.y()}, static_cast<int>(result.row)});
            } else {
                auto pose = landmark_measurements.at(result.col).to_xy();
                measurements.push_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{pose.x(), pose.y()}, -1});
            }
        }

        // correction
        ekf_slam_.correct(measurements);
    });

    auto ekf_prediction_period_ms = static_cast<size_t>(conf.ekf_prediction_period * 1000.0f);
    ekf_prediction_timer_ = create_wall_timer(std::chrono::milliseconds(ekf_prediction_period_ms),
                                             std::bind(&Controller::ekf_prediction_timer_callback,
                                             this));

    auto visualization_period_ms = static_cast<size_t>(conf.visualization_period * 1000.0f);
    visualization_timer_ = create_wall_timer(std::chrono::milliseconds(visualization_period_ms),
                                             std::bind(&Controller::visualization_timer_callback,
                                             this));
}


Controller::~Controller() {
    ekf_prediction_timer_->cancel();
    visualization_timer_->cancel();
}


void Controller::ekf_prediction_timer_callback() {
    auto linear_speed = gamepad_handler_.get_right_x();
    auto angular_speed = gamepad_handler_.get_left_y();
    ekf_slam_.predict(linear_speed, angular_speed, conf_.ekf_prediction_period);
}


void Controller::visualization_timer_callback() {
    visualization_engine_.draw_robot(simulation_engine_.get_robot_pose());
    visualization_engine_.draw_landmarks(conf_.landmarks);
    visualization_engine_.draw_landmark_measurements(simulation_engine_.get_landmark_measurements());

    auto agent = ekf_slam_.get_agent_state();
    visualization_engine_.draw_estimated_robot(rtl::RigidTf3f{rtl::Rotation3f{0, 0, agent.rotation().rotAngle()},
                                                              rtl::Translation3f{agent.translation().trVecX(),
                                                                                    agent.translation().trVecY(),
                                                                                    0}});

    const auto states = ekf_slam_.get_state_vector_matrix();
    const auto cov = ekf_slam_.get_covariant_matrix_();

    cv::Mat cov_img(cov.rowNr(), cov.colNr(), CV_32FC1, cv::Scalar(1.0f));
    cv::eigen2cv(cov.data(), cov_img);

    visualization_engine_.draw_covariance_matrix(cov_img);

    auto reduced_cov = rtl::Matrix33f();
    reduced_cov.setRow(0, rtl::Vector3f{cov.getElement(0,0), cov.getElement(0,1), cov.getElement(0,2)});
    reduced_cov.setRow(1, rtl::Vector3f{cov.getElement(1,0), cov.getElement(1,1), cov.getElement(1,2)});
    reduced_cov.setRow(2, rtl::Vector3f{cov.getElement(2,0), cov.getElement(2,1), cov.getElement(2,2)});
    visualization_engine_.draw_pose_with_covariance(rtl::Vector3f{states.getElement(0,0),states.getElement(1,0),0.0f},
                                                    states.getElement(2,0),
                                                    reduced_cov);

    auto slam_landmarks = ekf_slam_.get_landmarks();
    std::vector<rtl::Vector3f> slam_landmarks_v;
    for (const auto& landmark : slam_landmarks) {
        slam_landmarks_v.emplace_back(rtl::Vector3f{landmark.translation().trVecX(), landmark.translation().trVecY(), 0.0f});
    }
    visualization_engine_.draw_estimated_landmarks(slam_landmarks_v);
}