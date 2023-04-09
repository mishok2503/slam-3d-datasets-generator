#pragma once

#include <vector>

#include "mutil/mutil.h"

struct TLidarPoint {
    enum Type {
        UNKNOWN,
        POINT,
        MAX // out-of-sight measurement
    };

    Type type = UNKNOWN;
    mutil::Vector3 data;
};

class ILidar {
protected:
    [[nodiscard]] virtual std::vector<mutil::Vector3> GetPointsImpl() const = 0;

public:
    [[nodiscard]] const std::vector<mutil::Vector3> &GetPoints() const {
        static auto res = GetPointsImpl();
        return res;
    }

    [[nodiscard]] virtual float GetMaxDepth() const = 0;
    [[nodiscard]] virtual float GetVarCoef() const = 0;
    [[nodiscard]] virtual unsigned GetPointsCount() const = 0;

    virtual ~ILidar() = default;
};
