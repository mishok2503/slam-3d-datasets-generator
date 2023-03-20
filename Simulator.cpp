#include "Simulator.h"

#include "Robot.h"
#include "FibonacciLidar.h"

void Simulator::Run(unsigned steps, std::ostream& os) {
    os << steps << '\n';
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = Robot.EmulateLidar(Map);
        os << pointsCloud.size() << '\n';
        for (const auto &point: pointsCloud) {
            os << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        auto const& [pos, rot] = Robot.Move(); // TODO: write odometry
    }
}
