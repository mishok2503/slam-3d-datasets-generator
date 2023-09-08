#pragma once

#include "Map.h"

class TCubeMapGenerator : public IMapGenerator {
private:
    const TMapSize Size;

public:
    TMap Generate() override {
        auto res = TMap{Size}; // TODO: use isFilled param
        for (unsigned i = 0; i < Size.X; ++i) {
            for (unsigned j = 0; j < Size.Y; ++j) {
                for (unsigned k = 0; k < Size.Z; ++k) {
                    if (!i || !j || !k || i == Size.X - 1 || j == Size.Y - 1 || k == Size.Z - 1) {
                        res.SetCell(i, j, k, true);
                    }
                }
            }
        }
        return res;
    }

    explicit TCubeMapGenerator(TMapSize size) : Size(size) {}
};
