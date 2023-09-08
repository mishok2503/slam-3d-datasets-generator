#pragma once

#include "ErrorModel.h"

#include "util.h"

class TNormalErrorModel : public IErrorModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;
    const float QualityCoef;

public:
    TNormalErrorModel(float posCoef, float rotCoef, float lidarCoef, float qualityCoef) :
            PosCoef(posCoef), RotCoef(rotCoef), LidarCoef(lidarCoef), QualityCoef(qualityCoef) {}

    [[nodiscard]] mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, RotCoef, false);
    }

    [[nodiscard]] mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        for (int i = 0; i < 3; ++i) {
            float variance = std::abs(rotDelta[i] * RotCoef);
            result[i] += std::normal_distribution<float>{0, variance}(GetRandGen());
        }
        return result;
    }

    [[nodiscard]] mutil::Vector3 AddLidarError(const mutil::Vector3 &point, float quality) const override {
        return AddVectorError(point, LidarCoef * (1 + (1 - quality) * QualityCoef), false);
    }

    void Write(TWriter &writer, const char *key) const override {
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
