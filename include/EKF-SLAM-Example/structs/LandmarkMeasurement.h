#pragma once

struct LandmarkMeasurement {
    rtl::RigidTf3f sensor_pose = {};
    float pitch = 0.0f;
    float yaw = 0.0f;
    float range = 0.0f;
};
