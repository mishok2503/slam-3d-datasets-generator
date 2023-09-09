#pragma once

#include <random>
#include "mutil/mutil.h"

inline std::mt19937& GetRandGen() {
    constexpr int SEED = 239;
    static std::mt19937 randomGenerator{SEED};
    return randomGenerator;
}

inline mutil::Matrix3 GetRotationMatrix(mutil::Vector3 eulerAngles) {
    float a = eulerAngles.x, b = eulerAngles.y, c = eulerAngles.z;
    return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c),  sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
    };
}

inline mutil::Vector3 AddVectorError(const mutil::Vector3 &delta, const float variance, bool is2D = false) {
    std::normal_distribution<float> normalDistribution{0, variance};
    return delta + mutil::Vector3{
            normalDistribution(GetRandGen()),
            normalDistribution(GetRandGen()),
            is2D ? 0 : normalDistribution(GetRandGen())
    };
}

