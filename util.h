#pragma once

#include "mutil/mutil.h"

inline mutil::Matrix3 GetRotationMatrixInv(mutil::Vector3 eulerAngles) {
    float a = eulerAngles.x, b = eulerAngles.y, c = eulerAngles.z;
    return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c),  sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
    }.inverse();
}

inline mutil::Vector3 AddPositionError(const mutil::Vector3& vector, const float coef, std::mt19937& gen) {
    float variance = vector.length() * coef;
    std::normal_distribution<float> normalDistribution{0, variance};
    return vector + mutil::Vector3{
            normalDistribution(gen),
            normalDistribution(gen),
            normalDistribution(gen)
    };
}

inline mutil::Vector3 AddRotationError(mutil::Vector3 angles, const float coef, std::mt19937& gen) {
    for (int i = 0; i < 3; ++i) {
        float variance = std::abs(angles[i] * coef);
        angles[i] += std::normal_distribution<float>{0, variance}(gen);
    }
    return angles;
}
