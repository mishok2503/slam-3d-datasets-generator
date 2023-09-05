#pragma once

#include "Lidar.h"

class TOneBeamLidar : public ILidar {
private:
    const float MaxDepth;
    const mutil::Vector3 Direction;

    [[nodiscard]] std::vector<mutil::Vector3> GetPointsImpl() const override {
        return {Direction, {cos(0.3f), sin(0.3f), 0}, {cos(-0.3f), sin(-0.3f), 0}};
    }

public:
    TOneBeamLidar(mutil::Vector3 direction, float maxDepth) : Direction(std::move(direction)),
                                                              MaxDepth(maxDepth) {}

    [[nodiscard]] float GetMaxDepth() const override {
        return MaxDepth;
    }

    [[nodiscard]] unsigned GetPointsCount() const override {
        return 3u;
    }
};
