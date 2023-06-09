#pragma once

#include "Lidar.h"

class TFibonacciLidar : public ILidar {
private:
    const unsigned PointsCount;
    const float MaxDepth;

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        constexpr float phi = 2.399963229728653;
        std::vector<mutil::Vector3> p(PointsCount);
        for (int i = 0; i < PointsCount; ++i) {
            float y = 1 - (i / static_cast<float>(PointsCount - 1)) * 2;
            float r = sqrt(1 - y * y);
            float t = phi * i;
            p[i] = mutil::Vector3(cos(t) * r, y, sin(t) * r);
        }
        return p;
    }

public:
    explicit TFibonacciLidar(unsigned pointsCount, float maxDepth) :
            PointsCount(pointsCount), MaxDepth(maxDepth) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] unsigned GetPointsCount() const override {
        return PointsCount;
    }
};
