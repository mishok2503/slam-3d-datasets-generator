#pragma once

#include "MapGenerator.h"

class Simulator {
private:
    TMap map;

public:
    Simulator(TMapSize size, std::unique_ptr<IMapGenerator> mapGenerator) : map(mapGenerator->generate(size)) {}

    void run(unsigned steps, unsigned lidarPointsCount, std::ostream& os);

};
