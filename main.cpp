#include "Simulator.h"
#include "MazeMapGenerator.h"
#include "Robot.h"
#include "SimpleLidar.h"
#include "Lidar2D.h"
#include "UniformErrorModel.h"
#include "NormalErrorModel.h"
#include "ErrorModel2D.h"

#include <fstream>

int main() {
    constexpr TMapSize MAP_SIZE{7, 7, 5};
    constexpr unsigned STEPS_COUNT = 701;
    constexpr unsigned LIDAR_POINTS_COUNT = 2500;


    std::unique_ptr<IMapGenerator> mapGenerator{new TMazeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT, 15)};
    std::unique_ptr<TRobotBuilder> robotBuilder{new TRobotBuilder(std::move(lidar))};
    robotBuilder->SetSpeed(0.1);
    std::unique_ptr<IErrorModel> errorModel{
        new TErrorModel2D(0.05, 0.01, 0.01)
    };
//    std::unique_ptr<IErrorModel> errorModel{
//            new TUniformErrorModel(0.001, 0.001,
//                                   [](float r) { return r / 50; },
//                                   [](float theta) { return 0.01; },
//                                   [](float phi) { return 0.01; }
//            )};

    Simulator simulator(std::move(mapGenerator), std::move(robotBuilder), std::move(errorModel));

    std::ofstream dataFile("result.json");
    std::ofstream groundTruthFile("ground_truth.json");
    simulator.Run(STEPS_COUNT, dataFile, groundTruthFile);

    return 0;
}
