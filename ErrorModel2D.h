#pragma once

#include "ErrorModel.h"

#include "util.h"

template<bool IsLidar2D = false>
class TErrorModel2D : public IErrorModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;
    const float QualityCoef;

public:
    TErrorModel2D(float posCoef, float rotCoef, float lidarCoef, float qualityCoef = 10) : PosCoef(posCoef), RotCoef(rotCoef),
                                                                       LidarCoef(lidarCoef), QualityCoef(qualityCoef) {}

    [[nodiscard]] mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, PosCoef, true);
    }

    [[nodiscard]] mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        float variance = std::abs(rotDelta.z * RotCoef);
        result.z += std::normal_distribution<float>{0, variance}(GetRandGen());
        return result;
    }

    [[nodiscard]] mutil::Vector3 AddLidarError(const mutil::Vector3 &point, float quality) const override {
        return AddVectorError(point, LidarCoef * (1 + (1 - quality) * QualityCoef), IsLidar2D);
    }

    void Write(TWriter &writer, const char *key) const override {
        writer.Key(key);
        writer.StartObject();
        writer.Key("type");
        writer.String("2d_error");
        writeKeyDouble(writer, "position_coef", PosCoef);
        writeKeyDouble(writer, "rotation_coef", RotCoef);
        writeKeyDouble(writer, "lidar_coef", LidarCoef);
        writeKeyDouble(writer, "quality_coef", QualityCoef);
        writer.EndObject();
    }
};