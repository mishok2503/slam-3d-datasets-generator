#pragma once

#include <iostream>
#include <vector>
#include <optional>

#include "Cell.h"
#include "mutil.h"

struct TMapSize {
    const unsigned nx, ny, nz;

    bool Check(unsigned x, unsigned y, unsigned z) {
        return x < nx && y < ny && z < nz;
    }
};

class TMap {
public:
    using TRow = std::vector<TCell>;
    using TPlane = std::vector<TRow>;
    using TMapData = std::vector<TPlane>;
private:
    TMapData map;
    TMapSize size;

public:
    explicit TMap(TMapSize size) :
        map(TMapData(size.nx, TPlane(size.ny, TRow(size.nz)))),
        size(size) {}

    void SetCell(unsigned x, unsigned y, unsigned z, bool isOccupied) {
        if (!size.Check(x, y, z)) {
            std::cerr << "ERROR: out of borders!\n";
        }
        map[x][y][z].isOccupied = isOccupied;
    }

    [[nodiscard]] TMapSize GetSize() const {
        return size;
    }

    [[nodiscard]] mutil::Vector3 GetFreeCell() const {
        while (true) { // TODO: endless loop
            int x = rand() % size.nx; // TODO: norm rand
            int y = rand() % size.ny;
            int z = rand() % size.nz;
            if (!map[x][y][z].isOccupied) {
                return mutil::Vector3(x + 0.5, y + 0.5, z + 0.5);
            }
        }
    }

    [[nodiscard]] std::optional<float> GetDistance(mutil::Vector3 pos, mutil::Vector3 dir) const {
        constexpr float dt = 0.01;
        auto check = [this](mutil::Vector3 p) {
            for (int i=0; i < size.nx; ++i) {
                for (int j=0; j < size.ny; ++j) {
                    for (int k=0; k < size.nz; ++k) {
                        if (!map[i][j][k].isOccupied) continue;
                        if (i < p[0] && p[0] < i + 1 &&
                            j < p[1] && p[1] < j + 1 &&
                            k < p[2] && p[2] < k + 1) {
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
