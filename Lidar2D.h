#pragma once

#include "Lidar.h"

class TLidar2D : public ILidar {
private:
    const unsigned PointsCount;
    const float MaxDepth;

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        std::vector<mutil::Vector3> res(PointsCount);

        const float da = 2 * M_PI / PointsCount;

        float a = 0;
        for (auto &point: res) {
            point = {sin(a), cos(a), 0}; // maybe swap x, y
            a += da;
        }
        return res;
    }

public:
    TLidar2D(unsigned pointsCount, float maxDepth) :
        PointsCount(pointsCount), MaxDepth(maxDepth) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] unsigned GetPointsCount() const override {
        return PointsCount;
    }
};
