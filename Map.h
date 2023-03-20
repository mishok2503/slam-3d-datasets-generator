#pragma once

#include <iostream>
#include <vector>
#include <optional>
#include <random>
#include <stdexcept>

#include "Cell.h"
#include "mutil.h"

struct TMapSize {
    const unsigned X, Y, Z;

    void Check(unsigned x, unsigned y, unsigned z) const {
        if (!(x < X && y < Y && z < Z)) {
            throw std::invalid_argument("Out of Map borders");
        }
    }
};

class TMap {
public:
    using TRow = std::vector<TCell>;
    using TPlane = std::vector<TRow>;
    using TMapData = std::vector<TPlane>;
private:
    TMapData Map;
    TMapSize Size;

public:
    explicit TMap(TMapSize size) :
        Map(TMapData(size.X, TPlane(size.Y, TRow(size.Y)))),
        Size(size) {}

    void SetCell(unsigned x, unsigned y, unsigned z, bool isOccupied) {
        Size.Check(x, y, z);
        Map[x][y][z].isOccupied = isOccupied;
    }

    [[nodiscard]] TMapSize GetSize() const {
        return Size;
    }

    [[nodiscard]] mutil::Vector3 GetFreeCell() const {
        std::random_device randomDevice;
        std::mt19937 randomGenerator{randomDevice()};
        std::uniform_int_distribution<unsigned> rx(0, Size.X - 1);
        std::uniform_int_distribution<unsigned> ry(0, Size.Y - 1);
        std::uniform_int_distribution<unsigned> rz(0, Size.Z - 1);
        while (true) {
            unsigned x = rx(randomGenerator);
            unsigned y = ry(randomGenerator);
            unsigned z = rz(randomGenerator);
            if (!Map[x][y][z].isOccupied) {
                return {x + 0.5f, y + 0.5f, z + 0.5f};
            }
        }
    }

    [[nodiscard]] std::optional<float> GetDistance(mutil::Vector3 pos, mutil::Vector3 dir) const {
        constexpr float dt = 0.01;
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
        for (float t=dt; t < 10; t += dt) {
            if (check(pos + dir * t)) {
                return {t};
            }
        }
        return {};
    }
};


class IMapGenerator {
public:
    virtual TMap Generate() = 0;

    virtual ~IMapGenerator() = default;
};
