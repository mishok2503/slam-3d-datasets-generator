#include "Simulator.h"
#include "CubeMapGenerator.h"
#include "Robot.h"
#include "SimpleLidar.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{10, 10, 8};
    constexpr unsigned STEPS_COUNT = 10;
    constexpr unsigned LIDAR_POINTS_COUNT = 4000;


    std::unique_ptr<IMapGenerator> mapGenerator{new TCubeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT)};
    std::unique_ptr<TRobotGenerator> robotGenerator{new TRobotGenerator(std::move(lidar))};


    Simulator simulator(std::move(mapGenerator), std::move(robotGenerator));

    std::ofstream file("result.txt");
    simulator.Run(STEPS_COUNT, file);
    file.close();

    return 0;
}
