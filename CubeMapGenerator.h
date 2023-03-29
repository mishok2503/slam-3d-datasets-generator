#pragma once

#include "Map.h"

class TCubeMapGenerator : public IMapGenerator {
private:
    const TMapSize Size;

public:
    TMap Generate() override {
        auto res = TMap{Size};
        for (unsigned i = 0; i < Size.X; ++i) {
            for (unsigned j = 0; j < Size.Y; ++j) {
                for (unsigned k = 0; k < Size.Z; ++k) {
                    if (!i || !j || !k || i == Size.X - 1 || j == Size.Y - 1 || k == Size.Z - 1) {
                        res.SetCell(i, j, k, true);
                    }
                }
            }
        }
        res.SetCell(2, 2, 2, true);
        res.SetCell(2, 3, 2, true);
        res.SetCell(3, 2, 2, true);
        res.SetCell(2, 2, 3, true);
        return res;
    }

    explicit TCubeMapGenerator(TMapSize size) : Size(size) {}
};
