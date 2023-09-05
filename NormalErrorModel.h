#pragma once

#include "ErrorModel.h"

class TNormalErrorModel : public IErrorModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;

    static constexpr int SEED = 239;
    mutable std::mt19937 RandomGenerator{SEED};

    mutil::Vector3 AddVectorError(const mutil::Vector3 &delta, const float coef) const {
        float variance = coef;
        std::normal_distribution<float> normalDistribution{0, variance};
        return delta + mutil::Vector3{
                normalDistribution(RandomGenerator),
                normalDistribution(RandomGenerator),
                normalDistribution(RandomGenerator)
        };
    }

public:
    TNormalErrorModel(float posCoef, float rotCoef, float lidarCoef) : PosCoef(posCoef), RotCoef(rotCoef),
                                                                       LidarCoef(lidarCoef) {}

    mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, RotCoef);
    }

    mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        for (int i = 0; i < 3; ++i) {
            float variance = std::abs(rotDelta[i] * RotCoef);
            result[i] += std::normal_distribution<float>{0, variance}(RandomGenerator);
        }
        return result;
    }

    mutil::Vector3 AddLidarError(const mutil::Vector3 &point, float quality) const override {
        return AddVectorError(point, LidarCoef * (1 + (1 - quality) * 10));
    }

    virtual void Write(TWriter &writer, const char *key) const override {
        writer.Key(key);
        writer.StartObject();
        writer.Key("type");
        writer.String("normal_error");
        writeKeyDouble(writer, "position_coef", PosCoef);
        writeKeyDouble(writer, "rotation_coef", RotCoef);
        writeKeyDouble(writer, "lidar_coef", LidarCoef);
        writer.EndObject();
    }
};
