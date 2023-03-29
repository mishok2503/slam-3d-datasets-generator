#pragma once

#include "Map.h"
#include "Robot.h"

class Simulator {
private:
    TMap Map;
    TRobot Robot;

public:
    Simulator(std::unique_ptr<IMapGenerator> mapGenerator, std::unique_ptr<TRobotBuilder> robotBuilder) :
        Map(mapGenerator->Generate()), Robot(robotBuilder->SetPosition(Map.GetFreeCell()).Build()) {}

    void Run(unsigned steps, std::ostream& dataOs, std::ostream& groundTruthOs);

};
