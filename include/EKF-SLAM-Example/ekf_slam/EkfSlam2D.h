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

#include "LandmarkND.h"
#include "AgentND.h"
#include "rtl/alg/kalman/Kalman.h"


template<typename dtype, size_t max_num_of_landmarks>
class EkfSlam2D {

    static constexpr size_t dimension = 2;
    static constexpr size_t agent_states = dimension + 1;

    static constexpr size_t kalman_state_vector_dim = agent_states + (max_num_of_landmarks * 2);
    static constexpr size_t kalman_measurement_vector_dim = 2; //(max_num_of_landmarks * 2);
    static constexpr size_t kalman_control_vector_dim = 2;

public:

    EkfSlam2D(float motion_noise, float measurement_noise)
            : motion_noise_{motion_noise}
            , measurement_noise_{measurement_noise}
            , kalman_{motion_noise_, measurement_noise_} {
        auto cov = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::zeros();
        for (size_t i = agent_states; i < kalman_state_vector_dim ; i+=1) {
            cov.setElement(i,i, std::numeric_limits<float>::max());
        }
        kalman_.set_covariance_matrix(cov);

        auto measurement_noise_mat = rtl::Matrix<kalman_measurement_vector_dim, kalman_measurement_vector_dim, dtype>::identity() * measurement_noise_;
        kalman_.set_measurement_noise_covariance_matrix(measurement_noise_mat);
    }

    void predict(dtype v, dtype w, dtype dt) {
        auto fi = kalman_.states().getElement(2, 0);

        auto A = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::identity();
        kalman_.set_transision_matrix(A);

        auto B = rtl::Matrix<kalman_state_vector_dim, kalman_control_vector_dim, dtype>::zeros();
        B.setElement(0, 0, dt * cos(fi));
        B.setElement(1, 0, dt * sin(fi));
        B.setElement(2, 1, dt);
        kalman_.set_control_matrix(B);

        auto control_vector = rtl::Matrix<kalman_control_vector_dim, 1, dtype>::zeros();
        control_vector.setElement(0, 0, v);
        control_vector.setElement(1, 0, w);

        auto G_jacobian = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::identity();
        G_jacobian.setElement(0, 2, abs(v) * dt * -sin(fi));
        G_jacobian.setElement(1, 2, abs(v) * dt * cos(fi));

        auto R_noise = rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, dtype>::identity() * motion_noise_;
        kalman_.set_process_noise_covariance_matrix(R_noise);

        kalman_.extended_predict(control_vector, G_jacobian);
    }

    void correct(const std::vector<LandmarkND<dimension, dtype>>& measurements) {
        std::vector<LandmarkND<dimension, dtype>> new_landmarks;
        std::vector<LandmarkND<dimension, dtype>> existing_landmarks;
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

    [[nodiscard]] AgentND<dimension, dtype> get_agent_state() const {
        auto x = kalman_.states().getElement(0, 0);
        auto y = kalman_.states().getElement(1, 0);
        auto fi = kalman_.states().getElement(2, 0);
        auto agent = AgentND<dimension, dtype>(rtl::TranslationND<dimension, dtype>{x, y},
                                               rtl::RotationND<dimension, dtype>{fi});
        return agent;
    };

    [[nodiscard]] std::vector<LandmarkND<dimension, dtype>> get_landmarks() const {
        return estimate_landmarks();
    }

    [[nodiscard]] const rtl::Matrix<kalman_state_vector_dim, 1, float>& get_state_vector_matrix() const {
        return kalman_.states();
    }

    [[nodiscard]] const rtl::Matrix<kalman_state_vector_dim, kalman_state_vector_dim, float>& get_covariant_matrix_() const {
        return kalman_.covariance();
    }

private:

    const float motion_noise_;
    const float measurement_noise_;
    rtl::Kalman<dtype, kalman_state_vector_dim, kalman_measurement_vector_dim, kalman_control_vector_dim> kalman_;

    std::vector<LandmarkND<dimension, dtype>> estimate_landmarks() const {

        std::vector<LandmarkND<dimension, dtype>> output;
        auto landmark_index = 0;
        const auto stat_mat = kalman_.states();
        const auto cov_mat = kalman_.covariance();
        for (size_t i = agent_states ; i < cov_mat.colNr() ; i+=2) {
            if (cov_mat.getElement(i  , i) != std::numeric_limits<float>::max() &&
                cov_mat.getElement(i+1, i+1) != std::numeric_limits<float>::max()) {
                output.template emplace_back(
                        LandmarkND<dimension, dtype>{
                                rtl::TranslationND<dimension, dtype>{stat_mat.getElement(i, 0),
                                                                     stat_mat.getElement(i+1, 0)},
                                landmark_index
                        });
            }
            landmark_index += 1;
        }
        return output;
    }

    void insert_landmarks(const std::vector<LandmarkND<dimension, dtype>>& landmarks) {
        auto stat_mat = kalman_.states();
        auto cov_mat = kalman_.covariance();

        size_t landmark_index = 0;
        for (size_t i = agent_states ; i < cov_mat.colNr() ; i+=2) {
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

    void update_landmarks(const std::vector<LandmarkND<dimension, dtype>>& landmarks) {
//        auto stat_mat = kalman_.states();
//        auto cov_mat = kalman_.covariance();
//        for (const auto& landmark : landmarks) {
//            size_t matrix_index = agent_states + landmark.id() * 2;
//            stat_mat.setElement(matrix_index, 0, landmark.translation().trVecX());
//            stat_mat.setElement(matrix_index+1, 0, landmark.translation().trVecY());
//            cov_mat.setElement(matrix_index, matrix_index, measurement_noise_);
//            cov_mat.setElement(matrix_index+1, matrix_index+1, measurement_noise_);
//        }
//        kalman_.set_states(stat_mat);
//        kalman_.set_covariance_matrix(cov_mat);

        auto robot = get_agent_state();
        auto robot_yaw = robot.rotation().rotAngle();
        for (const auto& landmark : landmarks) {
            size_t matrix_index = agent_states + landmark.id() * 2;
            auto d_x = landmark.translation().trVecX() - robot.translation().trVecX();
            auto d_y = landmark.translation().trVecY() - robot.translation().trVecY();
            auto q = pow(d_x, 2) + pow(d_y, 2);
            auto d = sqrt(q);

            auto z_measurement = rtl::Matrix<kalman_measurement_vector_dim, 1, dtype>::zeros();
            z_measurement.setElement(0, 0, d);
            z_measurement.setElement(1, 0, atan2(d_y, d_x) - robot_yaw);

            auto H_jacobian = rtl::Matrix<kalman_measurement_vector_dim, kalman_state_vector_dim, dtype>::zeros();
            H_jacobian.setColumn(0, rtl::VectorND<2, dtype>{-d*d_x, d_y});
            H_jacobian.setColumn(1, rtl::VectorND<2, dtype>{-d*d_y, -d_x});
            H_jacobian.setColumn(2, rtl::VectorND<2, dtype>{0.0f, -q});
            H_jacobian.setColumn(matrix_index, rtl::VectorND<2, dtype>{d*d_x, -d_y});
            H_jacobian.setColumn(matrix_index+1, rtl::VectorND<2, dtype>{d*d_y, d_x});
            H_jacobian /= q;

            kalman_.extended_correct(z_measurement, H_jacobian);
        }

        auto cov = kalman_.covariance();
        std::cout << " - Cov - - - - " << std::endl;
        for (size_t i = 0 ; i < cov.rowNr() ; i++) {
            for (size_t j = 0 ; j < cov.colNr() ; j++) {
                std::cout << cov.getElement(i, j) << " ";
            }
            std::cout << std::endl;
        }
    }
};

#endif //ROBOTICTEMPLATELIBRARY_EKFSLAM2D_H
