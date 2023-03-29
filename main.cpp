#include "Simulator.h"
#include "MazeMapGenerator.h"
#include "Robot.h"
#include "SimpleLidar.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{9, 9, 5};
    constexpr unsigned STEPS_COUNT = 2500;
    constexpr unsigned LIDAR_POINTS_COUNT = 3000;


    std::unique_ptr<IMapGenerator> mapGenerator{new TMazeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT, 15)};
    std::unique_ptr<TRobotGenerator> robotGenerator{new TRobotGenerator(std::move(lidar))};


    Simulator simulator(std::move(mapGenerator), std::move(robotGenerator));

    std::ofstream file("result.txt");
    simulator.Run(STEPS_COUNT, file);
    file.close();

    return 0;
}
