#pragma once

#include "Lidar.h"

class TSimpleLidar : public ILidar {
private:
    const unsigned PointsCount;
    const float MaxDepth;
    const float DownAngle; // in radians, from 0 to PI / 2
    const float UpAngle; // same

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        std::vector<mutil::Vector3> res(PointsCount);

        const unsigned layersCount = sqrt(PointsCount) / 4;
        const float da = (UpAngle + DownAngle) / layersCount;
        const float db = 2 * M_PI * layersCount / PointsCount;
        float a = -DownAngle, b = 0;
        for (auto &point: res) {
            point = {sin(b) * cos(a), cos(b) * cos(a), sin(a)}; // maybe swap x, y
            b += db;
            if (b > 2 * M_PI) {
                b = 0;
                a += da;
            }
        }
        return res;
    }

public:
    TSimpleLidar(unsigned pointsCount, float maxDepth, float downAngle = M_PI / 3,
                 float upAngle = M_PI / 3) :
            PointsCount(pointsCount), MaxDepth(maxDepth), DownAngle(downAngle), UpAngle(upAngle) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] unsigned GetPointsCount() const override {
        return PointsCount;
    }
};
