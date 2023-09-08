#pragma once

#include "ErrorModel.h"

#include <utility>
#include "util.h"

class TUniformErrorModel : public IErrorModel {
public:
    using TErrorFunction = std::function<float(float)>;

private:
    mutable std::uniform_real_distribution<float> AngleDistribution;
    mutable std::uniform_real_distribution<float> PositionDistribution;

    TErrorFunction ErrorFromRadius;
    TErrorFunction ErrorFromTheta; // from 0 to PI
    TErrorFunction ErrorFromPhi; // from -PI to PI

    static mutil::Vector3 AddUniformError(mutil::Vector3 vec, std::uniform_real_distribution<float> &distribution) {
        for (int i = 0; i < 3; ++i) {
            vec[i] += distribution(GetRandGen());
        }
        return vec;
    }

    static float ApplyErrorFunction(float value, const TErrorFunction &func) {
        float error = func(value);
        return value + std::uniform_real_distribution<float>{-error, error}(GetRandGen());
    }

public:
    TUniformErrorModel(float rotError, float posError, TErrorFunction errorFromRadius, TErrorFunction errorFromTheta, TErrorFunction errorFromPhi)
            : AngleDistribution(-rotError, rotError), PositionDistribution(-posError, posError),
              ErrorFromRadius(std::move(errorFromRadius)), ErrorFromPhi(std::move(errorFromPhi)), ErrorFromTheta(std::move(errorFromTheta)) {}

    mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddUniformError(posDelta, PositionDistribution);
    }

    mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        return AddUniformError(rotDelta, AngleDistribution);
    }

    mutil::Vector3 AddLidarError(const mutil::Vector3 &point, float quality) const override {
        // translate to the spherical coordinate system
        auto r = point.length();
        float theta = std::atan2f(std::hypot(point.x, point.y), point.z);
        float phi = std::atan2f(point.y, point.x);
        // apply error
        r = ApplyErrorFunction(r, ErrorFromRadius);
        theta = ApplyErrorFunction(theta, ErrorFromTheta);
        phi = ApplyErrorFunction(phi, ErrorFromPhi);
        // translate back
        return r * mutil::Vector3{
                std::sin(theta) * std::cos(phi),
                std::sin(theta) * std::sin(phi),
                std::cos(theta)
        };
    }

    void Write(TWriter &writer, const char *key) const override {
        writer.Key(key);
        writer.StartObject();
        writer.Key("type");
        writer.String("uniform_error");
        writeKeyDouble(writer, "rotation_error", AngleDistribution.b());
        writeKeyDouble(writer, "position_error", PositionDistribution.b());
        // TODO: write functions
        writer.EndObject();
    }
};
