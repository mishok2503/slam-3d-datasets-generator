#pragma once

#include <array>
#include <iostream>
#include <vector>
#include <optional>
#include <random>
#include <stdexcept>

#include "Cell.h"
#include "mutil/mutil.h"
#include "Lidar.h"

std::mt19937& GetRandGen();


struct TMapSize {
    const unsigned X, Y, Z;

    void Check(unsigned x, unsigned y, unsigned z) const {
        if (!(x < X && y < Y && z < Z)) {
            throw std::invalid_argument("Out of Map borders");
        }
    }
};

inline std::ostream& operator <<(std::ostream& os, const TMapSize& size) {
    return os << size.X << ' ' << size.Y << ' ' << size.Z;
}

class TMap {
public:
    using TRow = std::vector<TCell>;
    using TPlane = std::vector<TRow>;
    using TMapData = std::vector<TPlane>;
private:
    TMapData Map;
    TMapSize Size;

    mutable std::uniform_int_distribution<unsigned> XDistribution;
    mutable std::uniform_int_distribution<unsigned> YDistribution;
    mutable std::uniform_int_distribution<unsigned> ZDistribution;
public:
    explicit TMap(TMapSize size, bool isFilled = false) :
        Map(TMapData(size.X, TPlane(size.Y, TRow(size.Y, {isFilled})))), Size(size),
        XDistribution(0, Size.X - 1), YDistribution(0, Size.Y - 1), ZDistribution(0, Size.Z - 1) {}

    void SetCell(unsigned x, unsigned y, unsigned z, bool isOccupied) {
        Size.Check(x, y, z);
        Map[x][y][z].isOccupied = isOccupied;
    }

    [[nodiscard]] bool IsCellOccupied(unsigned x, unsigned y, unsigned z) const {
        Size.Check(x, y, z);
        return Map[x][y][z].isOccupied;
    }

    TMapSize getSize() const {
        return Size;
    }

    [[nodiscard]] mutil::Vector3 GetFreeCell() const {
        while (true) {
            unsigned x = XDistribution(GetRandGen());
            unsigned y = YDistribution(GetRandGen());
            unsigned z = ZDistribution(GetRandGen());
            if (!Map[x][y][z].isOccupied) {
                return {x + 0.5f, y + 0.5f, z + 0.5f};
            }
        }
    }

    [[nodiscard]] std::pair<TLidarPoint::Type, float> GetDistance(mutil::Vector3 pos, mutil::Vector3 dir, const float maxDepth) const {
        constexpr float dt = 0.005;
        auto check = [this](mutil::Vector3 p) {
            for (int i=0; i < Size.X; ++i) {
                if (p.x < i) break;
                if (p.x > i + 1) continue;
                for (int j=0; j < Size.Y; ++j) {
                    if (p.y < j) break;
                    if (p.y > j + 1) continue;
                    for (int k=0; k < Size.Z; ++k) {
                        if (p.z < k) break;
                        if (p.z > k + 1) continue;
                        if (Map[i][j][k].isOccupied) {
                            return true;
                        }
                    }
                }
            }
            return false;
        };
        for (float t=dt; t < maxDepth; t += dt) {
            if (check(pos + dir * t)) {
                return {TLidarPoint::Type::POINT, t};
            }
        }
        return {TLidarPoint::Type::MAX, maxDepth};
    }
};


class IMapGenerator {
public:
    virtual TMap Generate() = 0;
    virtual ~IMapGenerator() = default;
};
