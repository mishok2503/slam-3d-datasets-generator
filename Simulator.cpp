#include "Simulator.h"

#include "Robot.h"
#include "FibonacciLidar.h"

namespace {
    std::ostream& operator <<(std::ostream& os, const mutil::Vector3& v) {
        return os << v[0] << ' ' << v[1] << ' ' << v[2];
    }
} // namespace

void Simulator::Run(unsigned steps, std::ostream& os) {
    os << steps << '\n';
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = Robot.EmulateLidar(Map);
        os << pointsCloud.size() << '\n';
        for (const auto &point: pointsCloud) {
            os << point.type << '\n';
            if (point.type == TLidarPoint::Type::POINT) {
                os << point.data << '\n';
            }
        }
        auto const& [pos, rot] = Robot.Move(Map);
        os << pos << ' ' << rot << '\n';
    }
}
