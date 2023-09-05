#pragma once

#include "ErrorModel.h"

template<bool Is2D>
class TErrorModel2D : public IErrorModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;

    const float QualityCoef;

    static constexpr int SEED = 239;
    mutable std::mt19937 RandomGenerator{SEED};

    mutil::Vector3 AddVectorError(const mutil::Vector3 &delta, const float coef, bool is2D = false) const {
        float variance = delta.length() * coef;
        std::normal_distribution<float> normalDistribution{0, variance};
        return delta + mutil::Vector3{
                normalDistribution(RandomGenerator),
                normalDistribution(RandomGenerator),
                is2D ? 0 : normalDistribution(RandomGenerator)
        };
    }

public:
    TErrorModel2D(float posCoef, float rotCoef, float lidarCoef, float qualityCoef = 10) : PosCoef(posCoef), RotCoef(rotCoef),
                                                                       LidarCoef(lidarCoef), QualityCoef(qualityCoef) {}

    mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, PosCoef);
    }

    mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        float variance = std::abs(rotDelta.z * RotCoef);
        result.z += std::normal_distribution<float>{0, variance}(RandomGenerator);
        return result;
    }

    mutil::Vector3 AddLidarError(const mutil::Vector3 &point, float quality) const override {
        return AddVectorError(point, LidarCoef * (1 + (1 - quality) * QualityCoef), Is2D);
    }

    virtual void Write(TWriter &writer, const char *key) const override {
        writer.Key(key);
        writer.StartObject();
        writer.Key("type");
        writer.String("2d_error");
        writeKeyDouble(writer, "position_coef", PosCoef);
        writeKeyDouble(writer, "rotation_coef", RotCoef);
        writeKeyDouble(writer, "lidar_coef", LidarCoef);
        writer.EndObject();
    }
};