#pragma once

#include "mutil.h"
#include "Map.h"
#include "Lidar.h"

class TRobot {
private:
    float Speed;
    mutil::Vector3 Position;
    mutil::Vector3 EulerAngles;
    mutil::Matrix3 RotationMatrix;
    mutil::Vector3 ForwardDirection;
    std::unique_ptr<ILidar> Lidar;

    float minDistance = INFINITY;
    const float minAllowedDistance = 0.2;

    static constexpr int SEED = 239;
    std::mt19937 RandomGenerator{SEED};
    std::uniform_real_distribution<float> TwoPiDistribution{0, 2 * M_PI};
public:

    TRobot(mutil::Vector3 position, std::unique_ptr<ILidar> lidar, float speed = 0.1,
           mutil::Vector3 eulerAngles = {}, mutil::Vector3 forwardDirection = {1, 0, 0}) :
           Speed(speed), Position(position), EulerAngles(eulerAngles), RotationMatrix(GetRotationMatrix(eulerAngles)),
           ForwardDirection(forwardDirection), Lidar(std::move(lidar)) {}

    std::vector<TLidarPoint> EmulateLidar(const TMap &map) {
        std::vector<TLidarPoint> res;
        auto points = Lidar->GetPoints();
        res.reserve(points.size());
        minDistance = INFINITY;
        for (const auto &point: points) {
            auto dist = map.GetDistance(Position, RotationMatrix * point, Lidar->GetMaxDepth());
            res.push_back({dist.first, point * dist.second});
            minDistance = std::min(minDistance, dist.second);
        }
        return res;
    }

    std::pair<mutil::Vector3, mutil::Vector3> Move(const TMap& map) {
        const auto prevAngles = EulerAngles;
        if (minDistance < minAllowedDistance) {
            const auto prevMinDistance = minDistance;
            const auto prevPosition = Position;
            while (true) {
                EulerAngles = {TwoPiDistribution(RandomGenerator), 0, 0};
                RotationMatrix = GetRotationMatrix(EulerAngles);
                Position += RotationMatrix * ForwardDirection * Speed;
                EmulateLidar(map);
                Position = prevPosition;
                if (minDistance > prevMinDistance) {
                    break;
                }
            }
        }
        Position += RotationMatrix * ForwardDirection * Speed;
        return {ForwardDirection * Speed, EulerAngles - prevAngles};
    }

    static mutil::Matrix3 GetRotationMatrix(mutil::Vector3 eulerAngles) {
        float a = eulerAngles.x, b = eulerAngles.y, c = eulerAngles.z;
        return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c),  sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
        }.inverse();
    }
};

class TRobotGenerator {
private:
    std::unique_ptr<ILidar> Lidar;

public:
    explicit TRobotGenerator(std::unique_ptr<ILidar> lidar) : Lidar(std::move(lidar)) {}

    TRobot Generate(mutil::Vector3 position = {}) {
        return {position, std::move(Lidar)};
    }
};