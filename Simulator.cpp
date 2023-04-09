#include "Simulator.h"

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "Robot.h"
#include "FibonacciLidar.h"

namespace {
    using TWriter = rapidjson::Writer<rapidjson::StringBuffer>;

    void writeKeyInt(TWriter& writer, const char* key, int value) {
        writer.Key(key);
        writer.Int(value);
    }

    void writeVector3(TWriter& writer, const char* key, const mutil::Vector3& vec) {
        writer.Key(key);
        writer.StartArray();
        writer.Double(vec.x);
        writer.Double(vec.y);
        writer.Double(vec.z);
        writer.EndArray();
    }

    void writeVectorUint(TWriter& writer, const std::vector<unsigned>& vec) {
        writer.StartArray();
        for (int i : vec) {
            writer.Int(i);
        }
        writer.EndArray();
    }

    void writeMap(TWriter& writer, const TMap& map) {
        writer.Key("map");
        writer.StartObject();
        const auto [x, y, z] = map.getSize();
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

    void writeRobot(TWriter& writer, const char* key, const mutil::Vector3& pos, const mutil::Vector3& angles) {
        writer.Key(key);
        writer.StartObject();
        writeVector3(writer, "position", pos);
        writeVector3(writer, "euler_angles", angles);
        writer.EndObject();
    }

    void writePointType(TWriter& writer, TLidarPoint::Type type) {
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
} // namespace

void Simulator::Run(unsigned steps, std::ostream& dataOs, std::ostream& groundTruthOs) {
    rapidjson::StringBuffer dataStringBuffer;
    rapidjson::StringBuffer truthStringBuffer;
    TWriter dataWriter(dataStringBuffer);
    TWriter truthWriter(truthStringBuffer);

    auto writeBoth = [&dataWriter, &truthWriter](auto func) {
        func(dataWriter);
        func(truthWriter);
    };

    writeBoth([](TWriter& writer) { writer.StartObject(); });

    constexpr int SEED = 239;
    std::mt19937 randomGenerator{SEED};

    writeMap(truthWriter, Map);
    writeRobot(truthWriter, "robot_start_position", Robot.GetPosition(), Robot.GetEulerAngles());

    writeBoth([&steps, this](TWriter& writer) {
        writer.Key("data");
        writer.StartObject();
        writeKeyInt(writer, "steps_count", steps);
        writeKeyInt(writer, "lidar_points_count", Robot.GetLidarPointsCount());
        writer.Key("measurements");
        writer.StartArray();
    });
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = Robot.EmulateLidar(Map);
        writeBoth([](TWriter& writer) {
            writer.StartObject(); // measurement elem
            writer.Key("lidar_data");
            writer.StartArray();
        });
        for (const auto &point: pointsCloud) {
            writeBoth([&point](TWriter& writer) {
                writer.StartObject(); // lidar_data elem
                writePointType(writer, point.type);
            });
            auto pointWithError = AddPositionError(point.data, Robot.GetLidarVarCoef(), randomGenerator);
            writeVector3(truthWriter, "coordinates", point.data);
            writeVector3(dataWriter, "coordinates", pointWithError);
            writeBoth([](TWriter& writer) { writer.EndObject(); }); // lidar_data elem
        }
        writeBoth([](TWriter& writer) { writer.EndArray(); }); // lidar_data
        auto const& [pos, rot] = Robot.Move(Map);
        auto posWithError = AddPositionError(pos, Robot.PositionVarCoef, randomGenerator);
        auto rotWithError = AddRotationError(rot, Robot.AnglesVarCoef  , randomGenerator);
        writeRobot(truthWriter, "odometry", pos, rot);
        writeRobot(dataWriter, "odometry", posWithError, rotWithError);
        writeBoth([](TWriter& writer) { writer.EndObject(); }); // measurement elem
    }
    writeBoth([](TWriter& writer) {
        writer.EndArray(); // measurements
        writer.EndObject(); // data
        writer.EndObject();
    });

    dataOs << dataStringBuffer.GetString() << std::endl;
    groundTruthOs << truthStringBuffer.GetString() << std::endl;
}
