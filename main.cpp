#include "Simulator.h"
#include "MazeMapGenerator.h"
#include "Robot.h"
#include "SimpleLidar.h"
#include "FibonacciLidar.h"
#include "UniformErrorModel.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{7, 7, 7};
    constexpr unsigned STEPS_COUNT = 80000;
    constexpr unsigned LIDAR_POINTS_COUNT = 1500;


    std::unique_ptr<IMapGenerator> mapGenerator{new TMazeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT, 15)};
    std::unique_ptr<TRobotBuilder> robotBuilder{new TRobotBuilder(std::move(lidar))};
    robotBuilder->SetSpeed(0.0002);
    std::unique_ptr<IErrorModel> errorModel{
            new TUniformErrorModel(0.02, 0.05,
                                   [](float r) { return r / 20; },
                                   [](float theta) { return 0.03; },
                                   [](float phi) { return 0.03; }
            )};

    Simulator simulator(std::move(mapGenerator), std::move(robotBuilder), std::move(errorModel));

    std::ofstream dataFile("result.json");
    std::ofstream groundTruthFile("ground_truth.json");
    simulator.Run(STEPS_COUNT, dataFile, groundTruthFile);

    return 0;
}
