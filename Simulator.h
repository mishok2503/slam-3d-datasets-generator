#pragma once

#include "Map/Map.h"
#include "Robot.h"
#include "ErrorModel/ErrorModel.h"

class Simulator {
private:
    TMap Map;
    TRobot Robot;
    std::unique_ptr<IErrorModel> ErrorModel;

public:
    Simulator(std::unique_ptr<IMapGenerator> mapGenerator, std::unique_ptr<TRobotBuilder> robotBuilder,
              std::unique_ptr<IErrorModel> errorModel) :
            Map(mapGenerator->Generate()), Robot(robotBuilder->SetPosition(Map.GetFreeCell()).Build()),
            ErrorModel(std::move(errorModel)) {}

    void Run(unsigned steps, std::ostream &dataOs, std::ostream &groundTruthOs);

};
