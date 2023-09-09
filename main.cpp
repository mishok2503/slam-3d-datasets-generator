#include "Simulator/Simulator.h"
#include "Map/MazeMapGenerator.h"
#include "Robot/Robot.h"
#include "Lidar/Simple.h"
#include "Lidar/Plain.h"
#include "Lidar/OneBeam.h"
#include "ErrorModel/SphericalUniform.h"
#include "ErrorModel/Normal.h"
#include "ErrorModel/Plain.h"

#include <fstream>

int main(int argc, char* argv[]) {
    constexpr TMapSize MAP_SIZE{7, 7, 5};
    constexpr unsigned STEPS_COUNT = 300;
    constexpr unsigned LIDAR_POINTS_COUNT = 2000;
    constexpr float LIDAR_MAX_DEPTH = 50;


    std::unique_ptr<IMapGenerator> mapGenerator{new TMazeMapGenerator(MAP_SIZE)};
    std::unique_ptr<ILidar> lidar{new TSimpleLidar(LIDAR_POINTS_COUNT, LIDAR_MAX_DEPTH)};
    std::unique_ptr<TRobotBuilder> robotBuilder{new TRobotBuilder(std::move(lidar))};
    robotBuilder->
         SetSpeed(0.1)
        .SetIsMove2D(true);
    std::unique_ptr<TErrorModel2D<false>> errorModel{
        new TErrorModel2D<false>(0.02, 0.001, 0.005)
    };
    errorModel->SetQualityFunction([](float x) {
        return x < 0.8 ? 1.f : 0.1f;
    });
    errorModel->SetQualityCoef(30);

    Simulator simulator(std::move(mapGenerator), std::move(robotBuilder), std::move(errorModel));

    std::string outputFilename = "result.json";
    if (argc >= 2) {
        outputFilename = argv[1];
    }
    std::ofstream dataFile(outputFilename);
    std::ofstream groundTruthFile("gt_" + outputFilename);
    simulator.Run(STEPS_COUNT, dataFile, groundTruthFile);

    return 0;
}
