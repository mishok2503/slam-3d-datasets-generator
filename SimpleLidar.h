#pragma once

#include "Lidar.h"

class TSimpleLidar : public ILidar {
private:
    const unsigned pointsCount;
    const float downAngle; // in radians, from 0 to PI / 2
    const float upAngle; // same

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        std::vector<mutil::Vector3> res(pointsCount);

        const unsigned pointsPerLayer = sqrt(pointsCount);
        const float da = (upAngle + downAngle) / pointsPerLayer;
        const float db = 2 * M_PI * pointsPerLayer / pointsCount;
        float a = -downAngle, b = 0;
        for (auto& point : res) {
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
    TSimpleLidar(unsigned pointsCount, float downAngle = M_PI / 3, float upAngle = M_PI / 3) :
        pointsCount(pointsCount), downAngle(downAngle), upAngle(upAngle) {}
};
