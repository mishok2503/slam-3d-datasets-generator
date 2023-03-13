#ifndef MAPGENERATOR_ROBOT_H
#define MAPGENERATOR_ROBOT_H

#include "Map.h"
#include "matrix.hpp"

struct RobotPos {
    Matrix pos{3, 1};
    Matrix rot;
    Matrix dir{3, 1};

    RobotPos(double x, double y, double z) : rot(feng::eye<double>( 3, 3 )) {
        dir[0][0] = 1;
        pos[0][0] = x;
        pos[1][0] = y;
        pos[2][0] = z;
    }

    void go(double speed) {
        pos += rot.inverse() * dir * speed;
    }
};

std::vector<Matrix> FibonacciSphere() {
    constexpr int N = 2000;
    constexpr double phi = 2.399963229728653;
    static auto res = [&]() {
        std::vector<Matrix> p(N);
        for (int i=0; i < N; ++i) {
            double y = 1 - (i / static_cast<double>(N - 1)) * 2;
            double r = sqrt(1 - y * y);
            double t = phi * i;
            double x = cos(t) * r;
            double z = sin(t) * r;
            p[i][0][0] = x;
            p[i][1][0] = y;
            p[i][2][0] = z;
        }
        return p;
    }();
    return res;
}

class Robot {
private:
    RobotPos pos;
    double speed;

public:
    Robot(RobotPos pos) : pos(pos) {}

    std::vector<Matrix> EmulateLidar(const Map& map) {
        const double dt = 0.01;
        std::vector<Matrix> points = FibonacciSphere();
        for (auto& point: points) {
            auto p = pos.rot.inverse() * point;
            double r = map.GetDistance(pos.pos, p);
            p *= r;
        }
        return points;
    }

    std::pair<Matrix, Matrix> Move() {
        auto t = pos.pos;
        pos.go(0.2);
        return {pos.pos - t, feng::zeros(3, 1)};
    }
};


#endif //MAPGENERATOR_ROBOT_H
