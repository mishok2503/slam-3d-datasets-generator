#ifndef MAPGENERATOR_CUBEMAPGENERATOR_H
#define MAPGENERATOR_CUBEMAPGENERATOR_H

#include "MapGenerator.h"

class CubeMapGenerator : public MapGenerator {
public:
    Map generate(MapSize size) override {
        auto res = Map(size);
        for (unsigned i = 0; i < size.nx; ++i) {
            for (unsigned j = 0; j < size.ny; ++j) {
                for (unsigned k = 0; k < size.nz; ++k) {
                    if (!i || !j || !k || i == size.nx - 1 || j == size.ny - 1 || k == size.nz - 1) {
                        res.SetCell(i, j, k, true);
                    }
                }
            }
        }
        return res;
    }
};

#endif //MAPGENERATOR_CUBEMAPGENERATOR_H
