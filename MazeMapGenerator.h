#pragma once

#include <functional>

#include "Map.h"
#include "CubeMapGenerator.h"

class TMazeMapGenerator : public IMapGenerator {
private:
    const TMapSize Size;

    static constexpr int SEED = 239;
    std::mt19937 RandomGenerator{SEED};
    const std::array<std::pair<int, int>, 4> Directions = {{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

public:
    TMap Generate() override {
        unsigned X = 2 * Size.X - 1;
        unsigned Y = 2 * Size.Y - 1;
        auto map = TMap{{X + 2, Y + 2, Size.Z}, true};
        std::vector<std::vector<bool>> maze(X, std::vector<bool>(Y));

        std::function<void(unsigned, unsigned)> dfs = [&](unsigned x, unsigned y) {
            maze[x][y] = true;
            auto directions = Directions;
            std::shuffle(directions.begin(), directions.end(), RandomGenerator);
            for (auto [dx, dy] : directions) {
                unsigned nx = x + 2 * dx;
                unsigned ny = y + 2 * dy;
                if (nx < X && ny < Y && !maze[nx][ny]) {
                    maze[x + dx][y + dy] = true;
                    dfs(nx, ny);
                }
            }
        };

        dfs(0, 0);
        for (unsigned i = 1; i < X - 1; ++i) {
            for (unsigned j = 1; j < Y - 1; ++j) {
                if (maze[i - 1][j - 1]) {
                    for (unsigned k = 1; k < Size.Z - 1; ++k) {
                        map.SetCell(i, j, k, false);
                    }
                }
            }
        }
        return map;
    }

    explicit TMazeMapGenerator(TMapSize size) : Size(size) {}
};
