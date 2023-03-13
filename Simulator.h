#ifndef MAPGENERATOR_SIMULATOR_H
#define MAPGENERATOR_SIMULATOR_H


#include "MapGenerator.h"

class Simulator {
private:
    Map map;

public:
    Simulator(MapSize size, std::unique_ptr<MapGenerator> mapGenerator) : map(mapGenerator->generate(size)) {}

    void run(size_t steps, std::ostream& os);

};


#endif //MAPGENERATOR_SIMULATOR_H
