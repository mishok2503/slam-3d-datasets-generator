#pragma once

#include "Map.h"
#include "Robot.h"

class Simulator {
private:
    TMap Map;
    TRobot Robot;

public:
    Simulator(std::unique_ptr<IMapGenerator> mapGenerator, std::unique_ptr<TRobotGenerator> robotGenerator) :
        Map(mapGenerator->Generate()), Robot(robotGenerator->Generate(Map.GetFreeCell())) {}

    void Run(unsigned steps, std::ostream& os);

};
