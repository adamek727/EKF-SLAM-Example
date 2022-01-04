#include <gtest/gtest.h>
#include <rtl/Transformation.h>
#include "EKF-SLAM-Example/ekf_slam/EkfSlam2D.h"

TEST(ekf_slam_test, test_1) {
    auto ekf_slam = EkfSlam2D<float, 2>(0.1, 0.1);
}

TEST(ekf_slam_test, prediction_test) {
    auto ekf_slam = EkfSlam2D<float, 2>(0.1, 0.1);
//    auto kalman = rtl::Kalman<float, 7, 2, 2>{0.1, 0.1};

    auto stats = rtl::Matrix<7, 1, float>::zeros();
    ekf_slam.set_state_matrix(stats);

    auto cov = rtl::Matrix<7, 7, float>::zeros();
    cov.setRow(0, rtl::VectorND<7, float>{0, 1, 2, 3, 4, 5, 6});
    cov.setRow(1, rtl::VectorND<7, float>{1, 2, 3, 4, 5, 6, 7});
    cov.setRow(2, rtl::VectorND<7, float>{2, 3, 4, 5, 6, 7, 8});
    cov.setRow(3, rtl::VectorND<7, float>{3, 4, 5, 6, 7, 8, 9});
    cov.setRow(4, rtl::VectorND<7, float>{4, 5, 6, 7, 8, 9, 10});
    cov.setRow(5, rtl::VectorND<7, float>{5, 6, 7, 8, 9, 10, 11});
    cov.setRow(6, rtl::VectorND<7, float>{6, 7, 8, 9, 10, 11, 12});
    ekf_slam.set_covariance_matrix(cov);

    auto Rt = rtl::Matrix<7, 7, float>::zeros();
    Rt.setElement(0, 0,0.1);
    Rt.setElement(1, 1,0.1);
    Rt.setElement(2, 2,0.1);
    ekf_slam.set_process_noise_matrix(Rt);

    auto d_states = rtl::Matrix<7, 1, float>::zeros();
    d_states.setElement(0, 0, 1.0);

    ekf_slam.predict(1, 0, 1);

    stats = ekf_slam.get_state_vector_matrix();
    cov = ekf_slam.get_covariant_matrix();

    std::cout << "Stats" << std::endl;
    for( int i = 0 ; i < stats.rowNr() ; i++) {
        for( int j = 0 ; j < stats.colNr() ; j++) {
            std::cout << stats.getElement(i, j) << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << "Cov" << std::endl;
    for( int i = 0 ; i < cov.rowNr() ; i++) {
        for( int j = 0 ; j < cov.colNr() ; j++) {
            std::cout << cov.getElement(i, j) << "\t";
        }
        std::cout << std::endl;
    }
}


TEST(ekf_slam_test, correction_test) {
    auto ekf_slam = EkfSlam2D<float, 2>(0.1, 0.1);
//    auto kalman = rtl::Kalman<float, 7, 2, 2>{0.1, 0.1};

    auto stats = rtl::Matrix<7, 1, float>::zeros();
    stats.setElement(3, 0, 1);
    stats.setElement(4, 0, 1);
    stats.setElement(5, 0, 2);
    ekf_slam.set_state_matrix(stats);

    auto cov = rtl::Matrix<7, 7, float>::zeros();
    cov.setRow(0, rtl::VectorND<7, float>{0, 1, 2, 3, 4, 5, 6});
    cov.setRow(1, rtl::VectorND<7, float>{1, 2, 3, 4, 5, 6, 7});
    cov.setRow(2, rtl::VectorND<7, float>{2, 3, 4, 5, 6, 7, 8});
    cov.setRow(3, rtl::VectorND<7, float>{3, 4, 5, 6, 7, 8, 9});
    cov.setRow(4, rtl::VectorND<7, float>{4, 5, 6, 7, 8, 9, 10});
    cov.setRow(5, rtl::VectorND<7, float>{5, 6, 7, 8, 9, 10, 11});
    cov.setRow(6, rtl::VectorND<7, float>{6, 7, 8, 9, 10, 11, 12});
    ekf_slam.set_covariance_matrix(cov);

    auto Qt = rtl::Matrix<2, 2, float>::zeros();
    Qt.setElement(0, 0, 0.1);
    Qt.setElement(1, 1, 0.1);
    ekf_slam.set_measurement_noise_matrix(Qt);

    auto d_states = rtl::Matrix<7, 1, float>::zeros();
    d_states.setElement(0, 0, 1.0);

    std::vector<LandmarkND<2, float>> measurements;
    measurements.emplace_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{1, 1}, 0});
    measurements.emplace_back(LandmarkND<2, float>{rtl::TranslationND<2, float>{2, 0}, 1});

    ekf_slam.correct(measurements);

    stats = ekf_slam.get_state_vector_matrix();
    cov = ekf_slam.get_covariant_matrix();

    std::cout << "Stats" << std::endl;
    for( int i = 0 ; i < stats.rowNr() ; i++) {
        for( int j = 0 ; j < stats.colNr() ; j++) {
            std::cout << stats.getElement(i, j) << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << "Cov" << std::endl;
    for( int i = 0 ; i < cov.rowNr() ; i++) {
        for( int j = 0 ; j < cov.colNr() ; j++) {
            std::cout << cov.getElement(i, j) << "\t";
        }
        std::cout << std::endl;
    }
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
