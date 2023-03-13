#include "Simulator.h"
#include "Robot.h"

void Simulator::run(size_t steps, std::ostream &os) {
    os << "STEPS COUNT: " << steps << '\n';
    auto [x, y, z] = map.GetFreeCell();
    Robot robot({x, y, z});

    for (int i=0; i < steps; ++i) {
        auto pointsCloud = robot.EmulateLidar(map);
        os << "POINTS COUNT: " << pointsCloud.size() << "\nPOINTS:\n";
        for (const auto &point: pointsCloud) {
            os << '\t' << point[0][0] << ' ' << point[1][0] << ' ' << point[2][0] << '\n';
        }
        auto const& [pos, rot] = robot.Move();
        os << "ODOMETRY:\n\t" << pos[0][0] << ' ' << pos[1][0] << ' ' << pos[2][0]
            << rot[0][0] << ' ' << rot[1][0] << ' ' << rot[2][0];
    }
}
