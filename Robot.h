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

public:

    TRobot(mutil::Vector3 position, std::unique_ptr<ILidar> lidar, float speed = 0.1,
           mutil::Vector3 eulerAngles = {}, mutil::Vector3 forwardDirection = {1, 0, 0}) :
           Speed(speed), Position(position), EulerAngles(eulerAngles), RotationMatrix(GetRotationMatrix(eulerAngles)),
           ForwardDirection(forwardDirection), Lidar(std::move(lidar)) {}

    std::vector<mutil::Vector3> EmulateLidar(const TMap &map) {
        auto points = Lidar->GetPoints();
        for (auto &point: points) {
            auto p = RotationMatrix.inverse() * point;
            float r = *map.GetDistance(Position, p); // TODO: optional
            point *= r;
        }
        return points;
    }

    std::pair<mutil::Vector3, mutil::Vector3> Move() {
        auto t = Position;
        Position += RotationMatrix.inverse() * ForwardDirection * Speed;
        Position.y -= Speed; // TODO: remove
        return {Position - t, {}};
    }

    static mutil::Matrix3 GetRotationMatrix(mutil::Vector3 eulerAngles) {
        float a = eulerAngles.x, b = eulerAngles.y, c = eulerAngles.z;
        return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c),  sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
        };
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