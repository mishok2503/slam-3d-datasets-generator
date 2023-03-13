#ifndef MAPGENERATOR_MAP_H
#define MAPGENERATOR_MAP_H

#include <iostream>
#include <vector>
#include "Cell.h"
#include "matrix.hpp"

struct MapSize {
    const unsigned nx, ny, nz;

    bool Check(unsigned x, unsigned y, unsigned z) {
        return x < nx && y < ny && z < nz;
    }
};

class Map {
public:
    using TRow = std::vector<Cell>;
    using TPlane = std::vector<TRow>;
    using TMap = std::vector<TPlane>;
private:
    TMap map;
    MapSize size;

public:
    Map(MapSize size) :
        map(TMap(size.nx, TPlane(size.ny, TRow(size.nz)))),
        size(size) {}

    void SetCell(unsigned x, unsigned y, unsigned z, bool isOccupied) {
        if (!size.Check(x, y, z)) {
            std::cerr << "ERROR: out of borders!\n";
        }
        map[x][y][z].isOccupied = isOccupied;
    }

    [[nodiscard]] MapSize GetSize() const {
        return size;
    }

    [[nodiscard]] std::tuple<double, double, double> GetFreeCell() const {
        while (true) {
            int x = rand() % size.nx;
            int y = rand() % size.ny;
            int z = rand() % size.nz;
            if (!map[x][y][z].isOccupied) {
                return std::make_tuple(x + 0.5, y + 0.5, z + 0.5);
            }
        }
    }

    double GetDistance(Matrix pos, Matrix dir) const {
        constexpr double dt = 0.01;
        auto check = [this](Matrix p) {
            for (int i=0; i < size.nx; ++i) {
                for (int j=0; j < size.ny; ++j) {
                    for (int k=0; k < size.nz; ++k) {
                        if (!map[i][j][k].isOccupied) continue;
                        if (i < p[0][0] && p[0][0] < i + 1 &&
                            j < p[1][0] && p[1][0] < j + 1 &&
                            k < p[2][0] && p[2][0] < k + 1) {
                            return true;
                        }
                    }
                }
            }
        };
        for (double t=dt; t < 10; t += dt) {
            if (check(pos + dir * t)) {
                return t;
            }
        }
    }
};

#endif //MAPGENERATOR_MAP_H
