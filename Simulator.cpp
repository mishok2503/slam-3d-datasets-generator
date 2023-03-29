#include "Simulator.h"

#include "Robot.h"
#include "FibonacciLidar.h"

namespace {
    std::ostream& operator <<(std::ostream& os, const mutil::Vector3& v) {
        return os << v[0] << ' ' << v[1] << ' ' << v[2];
    }
} // namespace

void Simulator::Run(unsigned steps, std::ostream& dataOs, std::ostream& groundTruthOs) {
    auto print = [&](const auto& v) {
        dataOs << v << '\n';
        groundTruthOs << v << '\n';
    };
    constexpr int SEED = 239;
    std::mt19937 randomGenerator{SEED};
    print(steps);
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = Robot.EmulateLidar(Map);
        print(pointsCloud.size());
        for (const auto &point: pointsCloud) {
            print(point.type);
            if (point.type == TLidarPoint::Type::POINT) {
                groundTruthOs << point.data << '\n';
                dataOs << AddPositionError(point.data, Robot.GetLidarVarCoef(), randomGenerator) << '\n';
            }
        }
        auto const& [pos, rot] = Robot.Move(Map);
        groundTruthOs << pos << ' ' << rot << '\n';
        dataOs << AddPositionError(pos, Robot.PositionVarCoef, randomGenerator) << ' '
               << AddRotationError(rot, Robot.AnglesVarCoef  , randomGenerator) << '\n';
    }
}
