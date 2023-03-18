#pragma once

#include "Map.h"

class IMapGenerator {
public:
    virtual TMap generate(TMapSize) = 0;

    virtual ~IMapGenerator() = default;
};
