#pragma once

#include "mutil/mutil.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "Map.h"
#include "Lidar.h"

using TWriter = rapidjson::Writer<rapidjson::StringBuffer>;

inline std::mt19937& GetRandGen() {
    constexpr int SEED = 239;
    static std::mt19937 randomGenerator{SEED};
    return randomGenerator;
}

inline mutil::Matrix3 GetRotationMatrix(mutil::Vector3 eulerAngles) {
    float a = eulerAngles.x, b = eulerAngles.y, c = eulerAngles.z;
    return mutil::Matrix3{
            cos(a) * cos(c) - sin(a) * cos(b) * sin(c), -cos(a) * sin(c) - sin(a) * cos(b) * cos(c),  sin(a) * sin(b),
            sin(a) * cos(c) + cos(a) * cos(b) * sin(c), -sin(a) * sin(c) + cos(a) * cos(b) * cos(c), -cos(a) * sin(b),
            sin(b) * sin(c), sin(b) * cos(c), cos(b)
    };
}

inline mutil::Vector3 AddVectorError(const mutil::Vector3 &delta, const float variance , bool is2D) {
    std::normal_distribution<float> normalDistribution{0, variance};
    return delta + mutil::Vector3{
            normalDistribution(GetRandGen()),
            normalDistribution(GetRandGen()),
            is2D ? 0 : normalDistribution(GetRandGen())
    };
}

inline void writeKeyInt(TWriter &writer, const char *key, int value) {
    writer.Key(key);
    writer.Int(value);
}

inline void writeKeyDouble(TWriter &writer, const char *key, double value) {
    writer.Key(key);
    writer.Double(value);
}

inline void writeVector3(TWriter &writer, const char *key, const mutil::Vector3 &vec) {
    writer.Key(key);
    writer.StartArray();
    writer.Double(vec.x);
    writer.Double(vec.y);
    writer.Double(vec.z);
    writer.EndArray();
}

inline void writeVectorUint(TWriter &writer, const std::vector<unsigned> &vec) {
    writer.StartArray();
    for (int i: vec) {
        writer.Int(i);
    }
    writer.EndArray();
}

inline void writeMap(TWriter &writer, const TMap &map) {
    writer.Key("map");
    writer.StartObject();
    const auto[x, y, z] = map.getSize();
    writer.Key("size");
    writeVectorUint(writer, {x, y, z});
    writer.Key("data");
    writer.StartArray();
    for (unsigned i = 0; i < x; ++i) {
        writer.StartArray();
        for (unsigned j = 0; j < y; ++j) {
            writer.StartArray();
            for (unsigned k = 0; k < z; ++k) {
                writer.Int(map.IsCellOccupied(i, j, k));
            }
            writer.EndArray();
        }
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
}

inline void writeRobot(TWriter &writer, const char *key, const mutil::Vector3 &pos, const mutil::Vector3 &angles) {
    writer.Key(key);
    writer.StartObject();
    writeVector3(writer, "position", pos);
    writeVector3(writer, "euler_angles", angles);
    writer.EndObject();
}

inline void writePointType(TWriter &writer, TLidarPoint::Type type) {
    writer.Key("type");
    switch (type) {
        case TLidarPoint::POINT:
            writer.String("point");
            break;
        case TLidarPoint::MAX:
            writer.String("maximum");
            break;
        default:
            writer.String("unknown");
    }
}
