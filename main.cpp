#include "Simulator.h"
#include "MazeMapGenerator.h"
#include "Robot.h"
#include "SimpleLidar.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{3, 3, 3};
    constexpr unsigned STEPS_COUNT = 2;
    constexpr unsigned LIDAR_POINTS_COUNT = 10;


    std::unique_ptr<IMapGenerator> mapGenerator{new TMazeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT, 15, 0.02)};
    std::unique_ptr<TRobotBuilder> robotBuilder{new TRobotBuilder(std::move(lidar))};
    robotBuilder->SetSpeed(0.1).SetPositionVarCoef(0.15).SetAnglesVarCoef(0.005);

    Simulator simulator(std::move(mapGenerator), std::move(robotBuilder));

    std::ofstream dataFile("result.json");
    std::ofstream groundTruthFile("ground_truth.json");
    simulator.Run(STEPS_COUNT, dataFile, groundTruthFile);

    return 0;
}
