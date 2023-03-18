#include "Simulator.h"
#include "CubeMapGenerator.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{10, 10, 8};
    constexpr unsigned STEPS_COUNT = 10;
    constexpr unsigned LIDAR_POINTS_COUNT = 4000;

    std::ofstream file("result.txt");

    std::unique_ptr<IMapGenerator> mapGenerator{new TCubeMapGenerator()};

    Simulator simulator(MAP_SIZE, std::move(mapGenerator));
    simulator.run(STEPS_COUNT, LIDAR_POINTS_COUNT, file);

    file.close();

    return 0;
}
