#pragma once

#include "Util/Math.h"
#include "Util/Json.h"
#include "QualityModel.h"
#include "ErrorModel.h"

class TNormalErrorModel : public IErrorModel, public TQualityModel {
private:
    const float PosCoef;
    const float RotCoef;
    const float LidarCoef;

public:
    TNormalErrorModel(float posCoef, float rotCoef, float lidarCoef, float qualityCoef) :
            PosCoef(posCoef), RotCoef(rotCoef), LidarCoef(lidarCoef) {}

    mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const override {
        return AddVectorError(posDelta, RotCoef, false);
    }

    mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const override {
        auto result = rotDelta;
        for (int i = 0; i < 3; ++i) {
            float variance = std::abs(rotDelta[i] * RotCoef);
            result[i] += std::normal_distribution<float>{0, variance}(GetRandGen());
        }
        return result;
    }

    TLidarPoint AddLidarError(const mutil::Vector3 &point) const override {
        auto [quality, coef] = GetQualityCoef();
        return {
            TLidarPoint::Type::POINT,
            AddVectorError(point, LidarCoef * coef, false),
            quality
        };
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
