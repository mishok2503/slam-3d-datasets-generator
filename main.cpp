#include <fstream>
#include "Simulator.h"
#include "CubeMapGenerator.h"

int main() {
    const MapSize mapSize{10, 10, 5};

    std::ofstream file("result.txt");

    Simulator simulator(mapSize, std::unique_ptr<MapGenerator>{new CubeMapGenerator()});
    simulator.run(20, file);

    file.close();

    return 0;
}
