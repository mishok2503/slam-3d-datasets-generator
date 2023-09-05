#pragma once

#include "Lidar.h"

class TOneBeamLidar : public ILidar {
private:
    const float MaxDepth;
    const mutil::Vector3 Direction;

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        return {
            {1, 0, 0},
            {0, 1, 0},
            {-1, 0, 0},
            {0, -1, 0}
        };
    }

public:
    TOneBeamLidar(float maxDepth) : MaxDepth(maxDepth) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] unsigned GetPointsCount() const override {
        return 4u;
    }
};
