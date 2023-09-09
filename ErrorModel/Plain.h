#pragma once

#include "ErrorModel.h"

#include <functional>
#include "Util/Math.h"
#include "Util/Json.h"

template<bool IsLidar2D = false>
class TErrorModel2D : public IErrorModel, public TQualityModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;

public:
    TErrorModel2D(float posCoef, float rotCoef, float lidarCoef)
        : PosCoef(posCoef), RotCoef(rotCoef), LidarCoef(lidarCoef) {}

    mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, PosCoef, true);
    }

    mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        float variance = std::abs(rotDelta.z * RotCoef);
        result.z += std::normal_distribution<float>{0, variance}(GetRandGen());
        return result;
    }

    TLidarPoint AddLidarError(const mutil::Vector3 &point) const override {
        auto [quality, coef] = GetQualityCoef();
        return {
            TLidarPoint::Type::POINT,
            AddVectorError(point, LidarCoef * coef, IsLidar2D),
            quality
        };
    }

    void Write(TWriter &writer, const char *key) const override {
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