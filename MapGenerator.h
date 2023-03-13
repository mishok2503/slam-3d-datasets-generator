#ifndef MAPGENERATOR_MAPGENERATOR_H
#define MAPGENERATOR_MAPGENERATOR_H

#include "Map.h"

class MapGenerator {
public:
    virtual Map generate(MapSize) = 0;

    virtual ~MapGenerator() = default;
};

#endif //MAPGENERATOR_MAPGENERATOR_H
