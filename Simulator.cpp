#include "Simulator.h"

#include "Robot.h"
#include "FibonacciLidar.h"

void Simulator::run(unsigned steps, unsigned lidarPointsCount, std::ostream& os) {
    auto freeCellCenter= map.GetFreeCell();
    std::unique_ptr<ILidar> robotsLidar{new TFibonacciLidar(lidarPointsCount)};
    TRobot robot(freeCellCenter, std::move(robotsLidar));

    os << steps << '\n';
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = robot.EmulateLidar(map);
        os << pointsCloud.size() << '\n';
        for (const auto &point: pointsCloud) {
            os << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        auto const& [pos, rot] = robot.Move(); // TODO: write odometry
    }
}
