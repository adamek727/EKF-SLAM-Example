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


template<typename dtype, size_t max_num_of_landmarks>
class EkfSlam2D {

    static constexpr size_t dimension = 2;
    static constexpr size_t state_vector_dim = 3 + (max_num_of_landmarks * 2);

public:

    EkfSlam2D() {

    }

    void predict(dtype lin_speed, dtype ang_speed, dtype dt) {

    }

    void correct(const std::vector<LandmarkND<dimension, dtype>>& measurements) {

    }

    [[nodiscard]] AgentND<dimension, dtype> get_agent_state() const {
        return AgentND<dimension, dtype>{rtl::VectorND<dimension, dtype>::zeros(),
                                         rtl::RotationND<dimension, dtype>{}};
    };

    [[nodiscard]] std::vector<LandmarkND<dimension, dtype>> get_landmarks() const {
        return std::vector<LandmarkND<dimension, dtype>>{};
    }

    [[nodiscard]] rtl::Matrix<state_vector_dim, 1, float>& get_state_vector_matrix() const {
        return state_vector_matrix_;
    }

    [[nodiscard]] rtl::Matrix<state_vector_dim, state_vector_dim, float>& get_covariant_matrix_() const {
        return covariant_matrix_;
    }

private:

    rtl::Matrix<state_vector_dim, 1, float> state_vector_matrix_;
    rtl::Matrix<state_vector_dim, state_vector_dim, float> covariant_matrix_;
};

#endif ROBOTICTEMPLATELIBRARY_EKFSLAM2D_H
