#pragma once

#include "Lidar.h"

class TSimpleLidar : public ILidar {
private:
    const unsigned pointsCount;
    const float MaxDepth;
    const float VarCoef;
    const float downAngle; // in radians, from 0 to PI / 2
    const float upAngle; // same

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        std::vector<mutil::Vector3> res(pointsCount);

        const unsigned layersCount = sqrt(pointsCount) / 2;
        const float da = (upAngle + downAngle) / layersCount;
        const float db = 2 * M_PI * layersCount / pointsCount;
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
    TSimpleLidar(unsigned pointsCount, float maxDepth, float varCoef, float downAngle = M_PI / 3, float upAngle = M_PI / 3) :
        pointsCount(pointsCount), MaxDepth(maxDepth), VarCoef(varCoef), downAngle(downAngle), upAngle(upAngle) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] float GetVarCoef() const override {
        return VarCoef;
    }
};
