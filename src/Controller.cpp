#include "EKF-SLAM-Example/Controller.h"

#include <chrono>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <rtl/Algorithms.h>

Controller::Controller(const Config& conf)
        : Node("ekf_slam_example")
        , conf_{conf}
        , simulation_engine_(static_cast<std::shared_ptr<Node>>(this), conf)
        , gamepad_handler_{static_cast<std::shared_ptr<Node>>(this), conf}
        , visualization_engine_{static_cast<std::shared_ptr<Node>>(this)}
        , ekf_slam_{conf.ekf_conf.motion_noise, conf_.ekf_conf.distance_noise} {

    gamepad_handler_.set_joystick_event_callback([&](){
        simulation_engine_.set_liner_speed(gamepad_handler_.get_right_x());
        simulation_engine_.set_angular_speed(gamepad_handler_.get_left_y());
    });

    simulation_engine_.set_measurement_callback([&]() {

        const std::lock_guard<std::mutex> lock(slam_mutex_);
        auto t_start = std::chrono::high_resolution_clock::now();

        auto landmark_measurements = simulation_engine_.get_landmark_measurements();
        auto slam_landmarks = ekf_slam_.get_landmarks();

        auto agent = ekf_slam_.get_agent_state();
        auto agent_tr_3d = rtl::Translation3f{rtl::Vector3f{agent.translation().trVecX(), agent.translation().trVecY(), 0.0f}};
        auto agent_rot_3d = rtl::Rotation3f{rtl::Quaternionf{0.0f, 0.0f, agent.rotation().rotAngle()}};
        auto agent_tf_3d = rtl::RigidTf3f{agent_rot_3d, agent_tr_3d};


        auto matched_measurements = match_measurements_to_landmarks(landmark_measurements, agent_tf_3d);
        ekf_slam_.correct(matched_measurements);

        auto t_stop = std::chrono::high_resolution_clock::now();
        auto duration = duration_cast<std::chrono::microseconds>(t_stop - t_start);
        std::cout << "Correction duration: " << duration.count() << "us" << std::endl;

//        auto cov = ekf_slam_.get_covariant_matrix();
//        std::cout << std::endl << " - Cov -" << std::endl;
//        for(int i = 0; i < cov.rowNr(); i++) {
//            for(int j = 0; j < cov.colNr(); j++) {
//                std::cout << cov.getElement(i, j) << "\t";
//            }
//            std::cout << std::endl;
//        }

        visualization_engine_.draw_measurements_wrt_estimated_robot(simulation_engine_.get_landmark_measurements(), agent);
        visualize_covariance();
    });

    auto ekf_prediction_period_ms = static_cast<size_t>(conf.ekf_conf.ekf_prediction_period * 1000.0f);
    ekf_prediction_timer_ = create_wall_timer(std::chrono::milliseconds(ekf_prediction_period_ms),
                                             std::bind(&Controller::ekf_prediction_timer_callback,
                                             this));

    auto visualization_period_ms = static_cast<size_t>(conf.visualization_config.visualization_period * 1000.0f);
    visualization_timer_ = create_wall_timer(std::chrono::milliseconds(visualization_period_ms),
                                             std::bind(&Controller::visualization_timer_callback,
                                             this));

    auto process_noise_mat = rtl::Matrix<EkfSlamType::kalman_state_vector_dim, EkfSlamType::kalman_state_vector_dim, float>::zeros();
    process_noise_mat.setElement(0, 0, conf.ekf_conf.motion_noise);
    process_noise_mat.setElement(1, 1, conf.ekf_conf.motion_noise);
    process_noise_mat.setElement(2, 2, conf.ekf_conf.motion_noise);
    ekf_slam_.set_process_noise_matrix(process_noise_mat);

    auto measurement_noise_mat = rtl::Matrix<EkfSlamType::kalman_measurement_vector_dim, EkfSlamType::kalman_measurement_vector_dim, float>::zeros();
    measurement_noise_mat.setElement(0, 0, powf(conf_.ekf_conf.distance_noise, 2.0f));
    measurement_noise_mat.setElement(1, 1, powf(conf_.ekf_conf.angle_noise, 2.0f));
    ekf_slam_.set_measurement_noise_matrix(measurement_noise_mat);
}


Controller::~Controller() {
    ekf_prediction_timer_->cancel();
    visualization_timer_->cancel();
}


void Controller::ekf_prediction_timer_callback() {
    const std::lock_guard<std::mutex> lock(slam_mutex_);
    auto linear_speed = gamepad_handler_.get_right_x() * conf_.sim_conf.linear_speed_coef;
    auto angular_speed = gamepad_handler_.get_left_y() * conf_.sim_conf.angular_speed_coef;
    ekf_slam_.predict(linear_speed, angular_speed, conf_.ekf_conf.ekf_prediction_period);

    auto robot_pose = simulation_engine_.get_robot_pose();
    auto agent_pose = ekf_slam_.get_agent_state();

    while (trajectory_gt_.size() > trajectory_history_size) { trajectory_gt_.pop_front();}
    trajectory_gt_.emplace_back(rtl::Vector3f{robot_pose.trVecX(), robot_pose.trVecY(), robot_pose.trVecZ()});

    while (trajectory_estimated_.size() > trajectory_history_size) { trajectory_estimated_.pop_front();}
    trajectory_estimated_.emplace_back(rtl::Vector3f{agent_pose.translation().trVecX(), agent_pose.translation().trVecY(), 0.0f});

    visualization_engine_.draw_real_trajectory(trajectory_gt_);
    visualization_engine_.draw_estimated_trajectory(trajectory_estimated_);
}


void Controller::visualization_timer_callback() {

    auto agent = ekf_slam_.get_agent_state();
    const auto states = ekf_slam_.get_state_vector_matrix();
    const auto cov = ekf_slam_.get_covariant_matrix();

    visualization_engine_.draw_robot(simulation_engine_.get_robot_pose());
    visualization_engine_.draw_landmarks(conf_.landmarks);
    visualization_engine_.draw_landmark_measurements(simulation_engine_.get_landmark_measurements());

    visualization_engine_.draw_estimated_robot(rtl::RigidTf3f{rtl::Rotation3f{0, 0, agent.rotation().rotAngle()},
                                                              rtl::Translation3f{agent.translation().trVecX(),
                                                                                    agent.translation().trVecY(),
                                                                                    0}});

    auto reduced_cov = rtl::Matrix33f();
    reduced_cov.setRow(0, rtl::Vector3f{cov.getElement(0,0), cov.getElement(0,1), cov.getElement(0,2)});
    reduced_cov.setRow(1, rtl::Vector3f{cov.getElement(1,0), cov.getElement(1,1), cov.getElement(1,2)});
    reduced_cov.setRow(2, rtl::Vector3f{cov.getElement(2,0), cov.getElement(2,1), cov.getElement(2,2)});
    visualization_engine_.draw_pose_with_covariance(rtl::Vector3f{states.getElement(0,0),states.getElement(1,0),0.0f},
                                                    states.getElement(2,0),
                                                    reduced_cov);

    auto slam_landmarks = ekf_slam_.get_landmarks();
    std::vector<std::pair<rtl::Vector3f, size_t>> slam_landmarks_v;
    for (const auto& landmark : slam_landmarks) {
        slam_landmarks_v.emplace_back(std::pair<rtl::Vector3f, size_t>
            {rtl::Vector3f{landmark.translation().trVecX(), landmark.translation().trVecY(), 0.0f}, landmark.id()});
    }
    visualization_engine_.draw_estimated_landmarks(slam_landmarks_v);
}


void Controller::visualize_covariance() {

    const auto states = ekf_slam_.get_state_vector_matrix();
    const auto cov = ekf_slam_.get_covariant_matrix();
    const auto inf = cov.inverted();
    auto agent_x = states.getElement(0, 0);
    auto agent_y = states.getElement(1, 0);
    size_t no_of_landmarks = (states.rowNr() - 3) / 2;

    cv::Mat cov_img(cov.rowNr(), cov.colNr(), CV_32FC1, cv::Scalar(0.0f));
    //cv::Mat inf_img(cov.rowNr(), cov.colNr(), CV_32FC1, cv::Scalar(0.0f));

    //cv::eigen2cv(cov.data(), cov_img);
    //cv::eigen2cv(inf.data(), inf_img);

    //visualization_engine_.draw_covariance_matrix(cov_img);
    //visualization_engine_.draw_information_matrix(inf_img);

    std::vector<VisualizationEngine::Correlation> correlations;
    for (size_t l = 0; l < no_of_landmarks ; l+=1) {
        size_t i = 3 + (l * 2);
        auto landmark_x = states.getElement(i, 0);
        auto landmark_y = states.getElement(i+1, 0);
        auto corr = sqrtf(powf(cov.getElement(i, 0),2) + powf(cov.getElement(i+1, 1),2));
        correlations.emplace_back(
                VisualizationEngine::Correlation{rtl::Vector3f{agent_x, agent_y, 0.0f},
                                                 rtl::Vector3f{landmark_x, landmark_y, 0.0f},
                                                 corr});
    }
    for (size_t l1 = 0 ; l1 < no_of_landmarks ; l1+=1) {
        for (size_t l2 = l1 + 1 ; l2 < no_of_landmarks ; l2+=1) {
            size_t i1 = 3 + (l1 * 2);
            size_t i2 = 3 + (l2 * 2);
            auto corr = sqrtf(powf(cov.getElement(i2, i1),2) + powf(cov.getElement(i2+1, i1+1),2));
            correlations.emplace_back(
                    VisualizationEngine::Correlation{rtl::Vector3f{states.getElement(i1,0), states.getElement(i1+1,0), 0.0f},
                                                     rtl::Vector3f{states.getElement(i2,0), states.getElement(i2+1,0), 0.0f},
                                                     corr});
        }
    }
    visualization_engine_.draw_correlations(correlations);
}


std::vector<LandmarkND<2, Controller::EkfSlamDtype>> Controller::match_measurements_to_landmarks(const std::vector<LandmarkMeasurement>& measurements, const rtl::RigidTf3f& agent_tf) {

    auto slam_landmarks = ekf_slam_.get_landmarks();

    auto cost = rtl::Matrix<num_of_landmarks, num_of_landmarks, EkfSlamDtype>::zeros();
    for (size_t m = 0 ; m < measurements.size() ; m+=1) {
        auto measurement = measurements.at(m);
        measurement.sensor_pose = agent_tf;
        const auto m_pose = measurement.to_xy();

        for (size_t s = 0 ; s < slam_landmarks.size() ; s+=1) {
            const auto slam_landmark = slam_landmarks.at(s);
            auto dist = sqrt(pow(slam_landmark.translation().trVecX() - m_pose.x(), 2.0f) +
                             pow(slam_landmark.translation().trVecY() - m_pose.y(), 2.0f));
            auto c = landmark_assign_distance - std::max(std::min(dist, landmark_assign_distance), 0.0f);
            cost.setElement(m, s, c);
        }
    }
    auto match_results = MunkresType::solve(cost, true);
    return mark_measurements_with_id(measurements, match_results, agent_tf);
}

std::vector<LandmarkND<2, Controller::EkfSlamDtype>> Controller::mark_measurements_with_id(const std::vector<LandmarkMeasurement>& measurements,
                                                                                           const std::array<Controller::MunkresType::Result, Controller::num_of_landmarks>& matched_measurements,
                                                                                           const rtl::RigidTf3f& agent_tf) {

    std::vector<LandmarkND<2, float>> measurements_with_id;
    for (size_t m = 0; m < measurements.size(); m += 1) {
        const auto result = matched_measurements.at(m);
        auto measurement = measurements.at(result.row);
        measurement.sensor_pose = agent_tf;
        auto pose = measurement.to_xy();
        if (result.cost > 0.0f) {
            measurements_with_id.push_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{pose.x(), pose.y()}, static_cast<int>(result.col)});
        } else {
            measurements_with_id.push_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{pose.x(), pose.y()}, -1});
        }
    }
    return measurements_with_id;
}