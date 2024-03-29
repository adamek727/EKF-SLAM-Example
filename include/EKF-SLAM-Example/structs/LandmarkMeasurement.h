#pragma once

struct LandmarkMeasurement {
    rtl::RigidTf3f sensor_pose = {};
    float pitch = 0.0f;
    float yaw = 0.0f;
    float range = 0.0f;

    [[nodiscard]] rtl::Vector3f to_xy() const {
        float sensor_r, sensor_p, sensor_y;
        sensor_pose.rot().rotQuaternion().rpy(sensor_r, sensor_p, sensor_y);
        float x = sensor_pose.trVecX() + range * cosf(yaw + sensor_y);
        float y = sensor_pose.trVecY() + range * sinf(yaw + sensor_y);
        return rtl::Vector3f {x, y, 0.0f};
    }

    [[nodiscard]] rtl::Vector3f to_xyz() const {
        return rtl::Vector3f::zeros();
    }
};
