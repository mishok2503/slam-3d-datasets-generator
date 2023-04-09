#pragma once

#include "mutil/mutil.h"
#include "Map.h"
#include "Lidar.h"
#include "util.h"

class TRobot {
private:
    float Speed;
    mutil::Vector3 Position;
    mutil::Vector3 EulerAngles;
    mutil::Matrix3 RotationMatrix;
    mutil::Vector3 ForwardDirection;
    std::unique_ptr<ILidar> Lidar;

    float MinDistance = INFINITY;
    const float MinAllowedDistance;

    static constexpr int SEED = 239;
    mutable std::mt19937 RandomGenerator{SEED};
    std::uniform_real_distribution<float> TwoPiDistribution{0, 2 * M_PI};

public:
    const float PositionVarCoef;
    const float AnglesVarCoef;

    TRobot(mutil::Vector3 position, std::unique_ptr<ILidar> lidar, float speed,
           mutil::Vector3 eulerAngles, mutil::Vector3 forwardDirection, float positionVarCoef, float anglesVarCoef) :
           Speed(speed), Position(position), EulerAngles(eulerAngles), RotationMatrix(GetRotationMatrixInv(eulerAngles)),
           ForwardDirection(forwardDirection), Lidar(std::move(lidar)), MinAllowedDistance(2 * speed),
           PositionVarCoef(positionVarCoef), AnglesVarCoef(anglesVarCoef) {}

    std::vector<TLidarPoint> EmulateLidar(const TMap &map) {
        std::vector<TLidarPoint> res;
        auto points = Lidar->GetPoints();
        res.reserve(points.size());
        MinDistance = INFINITY;
        for (const auto &point: points) {
            auto dist = map.GetDistance(Position, RotationMatrix * point, Lidar->GetMaxDepth());
            res.push_back({dist.first, point * dist.second});
            MinDistance = std::min(MinDistance, dist.second);
        }
        return res;
    }

    std::pair<mutil::Vector3, mutil::Vector3> Move(const TMap& map) {
        const auto prevAngles = EulerAngles;
        if (MinDistance < MinAllowedDistance) {
            const auto prevMinDistance = MinDistance;
            const auto prevPosition = Position;
            while (true) {
                EulerAngles = {
                        TwoPiDistribution(RandomGenerator),
                        TwoPiDistribution(RandomGenerator),
                        TwoPiDistribution(RandomGenerator)
                };
                RotationMatrix = GetRotationMatrixInv(EulerAngles);
                Position += RotationMatrix * ForwardDirection * Speed;
                EmulateLidar(map);
                Position = prevPosition;
                if (MinDistance > prevMinDistance) {
                    break;
                }
            }
        }
        Position += RotationMatrix * ForwardDirection * Speed;
        return {ForwardDirection * Speed, EulerAngles - prevAngles};
    }

    float GetLidarVarCoef() const {
        return Lidar->GetVarCoef();
    }

    mutil::Vector3 GetPosition() const {
        return Position;
    }

    mutil::Vector3 GetEulerAngles() const {
        return EulerAngles;
    }

    unsigned GetLidarPointsCount() const {
        return Lidar->GetPointsCount();
    }
};

class TRobotBuilder {
private:
    float Speed = 0.1;
    mutil::Vector3 Position = {};
    mutil::Vector3 EulerAngles = {};
    mutil::Vector3 ForwardDirection = {1, 0, 0};
    std::unique_ptr<ILidar> Lidar;
    float PositionVarCoef = 0.1;
    float AnglesVarCoef = 0.1;

public:
    explicit TRobotBuilder(std::unique_ptr<ILidar> lidar) : Lidar(std::move(lidar)) {}

    TRobotBuilder& SetSpeed(float speed) {
        Speed = speed;
        return *this;
    }

    TRobotBuilder& SetPosition(mutil::Vector3 position) {
        Position = std::move(position);
        return *this;
    }

    TRobotBuilder& SetEulerAngles(mutil::Vector3 eulerAngles) {
        EulerAngles = std::move(eulerAngles);
        return *this;
    }

    TRobotBuilder& SetForwardDirection(mutil::Vector3 forwardDirection) {
        ForwardDirection = std::move(forwardDirection);
        return *this;
    }

    TRobotBuilder& SetPositionVarCoef(float positionVarCoef) {
        if (positionVarCoef < 0) {
            throw std::invalid_argument("PositionVarCoef must be not less then 0");
        }
        PositionVarCoef = positionVarCoef;
        return *this;
    }

    TRobotBuilder& SetAnglesVarCoef(float anglesVarCoef) {
        if (anglesVarCoef < 0) {
            throw std::invalid_argument("AnglesVarCoef must be not less then 0");
        }
        AnglesVarCoef = anglesVarCoef;
        return *this;
    }

    TRobot Build() {
        return {
            std::move(Position),
            std::move(Lidar),
            Speed,
            std::move(EulerAngles),
            std::move(ForwardDirection),
            PositionVarCoef,
            AnglesVarCoef
        };
    }
};