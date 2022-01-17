// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>


#ifndef ROBOTICTEMPLATELIBRARY_EKFSLAM2D_H
#define ROBOTICTEMPLATELIBRARY_EKFSLAM2D_H


#include <cmath>
#include <iomanip>

#include "LandmarkND.h"
#include "AgentND.h"
#include "rtl/alg/kalman/Kalman.h"


template<typename dtype, size_t max_num_of_landmarks>
class EkfSlam2D {

public:

    static constexpr size_t filter_dim = 2;
    static constexpr size_t agent_states_dim = filter_dim + 1;
    static constexpr size_t kalman_state_vector_dim = agent_states_dim + (max_num_of_landmarks * 2);
    static constexpr size_t kalman_measurement_vector_dim = 2;
    static constexpr size_t kalman_control_vector_dim = 2;

    EkfSlam2D(float motion_noise, float measurement_noise)
            : motion_noise_{motion_noise}
            , measurement_noise_{measurement_noise}
            , kalman_{motion_noise_, measurement_noise_} {
        auto cov = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::zeros();
        for (size_t i = agent_states_dim; i < kalman_state_vector_dim ; i+=1) {
            cov.setElement(i,i, std::numeric_limits<float>::max());
        }
        kalman_.set_covariance_matrix(cov);

        auto measurement_noise_mat = rtl::Matrix<kalman_measurement_vector_dim, kalman_measurement_vector_dim, dtype>::identity() * measurement_noise_;
        kalman_.set_measurement_noise_covariance_matrix(measurement_noise_mat);
    }

    void predict(dtype v, dtype w, dtype dt) {

        auto fi = kalman_.states().getElement(2, 0);

        auto x_diff = rtl::Matrix<kalman_state_vector_dim, 1, dtype>::zeros();
        x_diff.setElement(0, 0, v * dt * cos(fi));
        x_diff.setElement(1, 0, v * dt * sin(fi));
        x_diff.setElement(2, 0, w * dt);

        auto G_jacobian = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::identity();
        G_jacobian.setElement(0, 2, v * dt * -sin(fi));
        G_jacobian.setElement(1, 2, v * dt * cos(fi));

        kalman_.extended_predict(x_diff, G_jacobian);
        normalize_agent_rotation();
    }


    void correct(const std::vector<LandmarkND<filter_dim, dtype>>& measurements) {

        std::vector<LandmarkND<filter_dim, dtype>> new_landmarks;
        std::vector<LandmarkND<filter_dim, dtype>> existing_landmarks;

        for (const auto& measurement : measurements) {
            if (measurement.id() == -1) {
                new_landmarks.push_back(measurement);
            } else {
                existing_landmarks.push_back(measurement);
            }
        }
        insert_landmarks(new_landmarks);
        update_landmarks(existing_landmarks);
    }

    void set_state_matrix(const rtl::Matrix<kalman_state_vector_dim, 1, dtype>& states) {
        kalman_.set_states(states);
    }

    void set_covariance_matrix(const rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>& cov) {
        kalman_.set_covariance_matrix(cov);
    }

    void set_process_noise_matrix(const rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype> process_noise) {
        kalman_.set_process_noise_covariance_matrix(process_noise);
    }

    void set_measurement_noise_matrix(const rtl::Matrix<kalman_measurement_vector_dim, kalman_measurement_vector_dim, dtype> measurements_noise) {
        kalman_.set_measurement_noise_covariance_matrix(measurements_noise);
    }

    [[nodiscard]] AgentND<filter_dim, dtype> get_agent_state() const {
        return AgentND<filter_dim, dtype>(rtl::TranslationND<filter_dim, dtype>{kalman_.states().getElement(0, 0),
                                                                                kalman_.states().getElement(1, 0)},
                                          rtl::RotationND<filter_dim, dtype>{kalman_.states().getElement(2, 0)});
    };

    [[nodiscard]] LandmarkND<filter_dim, dtype> get_landmark(size_t index) const {
        return estimate_landmarks(index).at(index);
    }

    [[nodiscard]] std::vector<LandmarkND<filter_dim, dtype>> get_landmarks() const {
        return estimate_landmarks();
    }

    [[nodiscard]] const rtl::Matrix<kalman_state_vector_dim, 1, float>& get_state_vector_matrix() const {
        return kalman_.states();
    }

    [[nodiscard]] const rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, float>& get_covariant_matrix() const {
        return kalman_.covariance();
    }

private:

    const float motion_noise_;
    const float measurement_noise_;
    rtl::Kalman<dtype, kalman_state_vector_dim, kalman_measurement_vector_dim, kalman_control_vector_dim> kalman_;

    std::vector<LandmarkND<filter_dim, dtype>> estimate_landmarks(int req_index = -1) const {

        std::vector<LandmarkND<filter_dim, dtype>> output;
        auto landmark_index = 0;
        const auto stat_mat = kalman_.states();
        const auto cov_mat = kalman_.covariance();
        for (size_t i = agent_states_dim ; i < cov_mat.colNr() ; i+=2) {
            if (cov_mat.getElement(i  , i) != std::numeric_limits<float>::max() &&
                cov_mat.getElement(i+1, i+1) != std::numeric_limits<float>::max()) {
                output.template emplace_back(
                        LandmarkND<filter_dim, dtype>{
                                rtl::TranslationND<filter_dim, dtype>{stat_mat.getElement(i, 0),
                                                                      stat_mat.getElement(i+1, 0)},
                                landmark_index
                        });
                if (landmark_index == req_index) {
                    break;
                }
            }
            landmark_index += 1;
        }
        return output;
    }

    void insert_landmarks(const std::vector<LandmarkND<filter_dim, dtype>>& landmarks) {
        auto stat_mat = kalman_.states();
        auto cov_mat = kalman_.covariance();

        size_t landmark_index = 0;
        for (size_t i = agent_states_dim ; i < cov_mat.colNr() ; i+=2) {
            if (cov_mat.getElement(i  , i) == std::numeric_limits<float>::max() &&
                cov_mat.getElement(i+1, i+1) == std::numeric_limits<float>::max()) {

                if (landmark_index >= landmarks.size()) {break;}

                // insert new landmark
                auto landmark = landmarks.at(landmark_index);
                stat_mat.setElement(i, 0, landmark.translation().trVecX());
                stat_mat.setElement(i+1, 0, landmark.translation().trVecY());
                cov_mat.setElement(i, i, measurement_noise_);
                cov_mat.setElement(i+1, i+1, measurement_noise_);
                landmark_index+=1;
            }
        }
        kalman_.set_states(stat_mat);
        kalman_.set_covariance_matrix(cov_mat);
    }

    void update_landmarks(const std::vector<LandmarkND<filter_dim, dtype>>& landmarks) {

        for (const auto& landmark : landmarks) {

            auto robot = get_agent_state();
            auto robot_yaw = robot.rotation().rotAngle();

            size_t matrix_index = agent_states_dim + landmark.id() * 2;
            auto d_x = landmark.translation().trVecX() - robot.translation().trVecX();
            auto d_y = landmark.translation().trVecY() - robot.translation().trVecY();
            auto q = powf(d_x, 2) + powf(d_y, 2);
            auto d = sqrtf(q);
            auto fi = atan2f(d_y, d_x);

            auto z_measurement = rtl::Matrix<kalman_measurement_vector_dim, 1, dtype>::zeros();
            z_measurement.setElement(0, 0, d);
            z_measurement.setElement(1, 0, normalize_angle(fi - robot_yaw));

            auto lm = get_landmark(landmark.id());
            auto z_map = rtl::Matrix<kalman_measurement_vector_dim, 1, dtype>::zeros();
            z_map.setElement(0, 0, sqrtf(powf(lm.translation().trVecX() - robot.translation().trVecX(), 2.0f) +
                                         powf(lm.translation().trVecY() - robot.translation().trVecY(), 2.0f)));
            z_map.setElement(1, 0, atan2(lm.translation().trVecY() - robot.translation().trVecY(),
                                         lm.translation().trVecX() - robot.translation().trVecX()) - robot_yaw);

            auto z_diff = z_measurement - z_map;
            z_diff.setElement(1, 0, normalize_angle(z_diff.getElement(1, 0)));

            auto H_jacobian = rtl::Matrix<kalman_measurement_vector_dim, kalman_state_vector_dim, dtype>::zeros();
            H_jacobian.setColumn(0, rtl::VectorND<2, dtype>{-d*d_x, d_y});
            H_jacobian.setColumn(1, rtl::VectorND<2, dtype>{-d*d_y, -d_x});
            H_jacobian.setColumn(2, rtl::VectorND<2, dtype>{0.0f, -q});
            H_jacobian.setColumn(matrix_index, rtl::VectorND<2, dtype>{d*d_x, -d_y});
            H_jacobian.setColumn(matrix_index+1, rtl::VectorND<2, dtype>{d*d_y, d_x});
            H_jacobian /= q;

            kalman_.extended_correct(z_diff, H_jacobian);
            normalize_agent_rotation();
        }
    }

    void normalize_agent_rotation() {
        auto states = kalman_.states();
        auto fi = states.getElement(2, 0);
        states.setElement(2, 0, normalize_angle(fi));
        kalman_.set_states(states);
    }

    float normalize_angle(const float angle) {
        return atan2f(sinf(angle), cosf(angle));
    }
};

#endif
