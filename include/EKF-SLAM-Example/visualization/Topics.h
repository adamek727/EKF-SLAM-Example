#pragma once

namespace topics {

    inline const std::string& PRESET() {
        static const std::string str("/ekf_slam_example");
        return str;
    }

    inline const std::string& ROBOT() {
        static const std::string str = PRESET() + "/robot";
        return str;
    }

    inline const std::string& LANDMARKS() {
        static const std::string str = PRESET() + "/landmarks";
        return str;
    }

    inline const std::string& LANDMARK_MEASUREMENTS() {
        static const std::string str = PRESET() + "/landmark_measurements";
        return str;
    }


}