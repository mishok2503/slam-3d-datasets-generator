#include "Simulator.h"

#include <array>

#include "util.h"

namespace {
    constexpr int SEED = 239;
    std::mt19937 randomGenerator{SEED};
    std::array<float, 4> qualities = {1, 1, 1, 0.1};
    std::uniform_int_distribution<int> choice{0, qualities.size() - 1};
}

void Simulator::Run(unsigned steps, std::ostream &dataOs, std::ostream &groundTruthOs) {
    rapidjson::StringBuffer dataStringBuffer;
    rapidjson::StringBuffer truthStringBuffer;
    TWriter dataWriter(dataStringBuffer);
    TWriter truthWriter(truthStringBuffer);

    auto writeBoth = [&dataWriter, &truthWriter](auto func) {
        func(dataWriter);
        func(truthWriter);
    };

    writeBoth([](TWriter &writer) { writer.StartObject(); });

    writeMap(truthWriter, Map);
    writeRobot(truthWriter, "robot_start_position", Robot.GetPosition(), Robot.GetEulerAngles());
    ErrorModel->Write(truthWriter, "error_model");

    writeBoth([&steps, this](TWriter &writer) {
        writer.Key("data");
        writer.StartObject();
        writeKeyInt(writer, "steps_count", steps);
        writeKeyInt(writer, "lidar_points_count", Robot.GetLidarPointsCount());
        writer.Key("measurements");
        writer.StartArray();
    });
    for (int i = 0; i < steps; ++i) {
        auto pointsCloud = Robot.EmulateLidar(Map);
        writeBoth([](TWriter &writer) {
            writer.StartObject(); // measurement elem
            writer.Key("lidar_data");
            writer.StartArray();
        });
        for (const auto &point: pointsCloud) {
            writeBoth([&point](TWriter &writer) {
                writer.StartObject(); // lidar_data elem
                writePointType(writer, point.type);
            });
            float quality = qualities[choice(randomGenerator)];
            auto pointWithError = ErrorModel->AddLidarError(point.data, quality);
            writeVector3(truthWriter, "coordinates", point.data);
            writeVector3(dataWriter, "coordinates", pointWithError);
            writeBoth([quality](TWriter &writer) { writeKeyDouble(writer, "quality", quality); });
            writeBoth([](TWriter &writer) { writer.EndObject(); }); // lidar_data elem
        }
        writeBoth([](TWriter &writer) { writer.EndArray(); }); // lidar_data
        auto const &[pos, rot] = Robot.Move(Map);
        auto posWithError = ErrorModel->AddPositionError(pos);
        auto rotWithError = ErrorModel->AddRotationError(rot);
        writeRobot(truthWriter, "odometry", pos, rot);
        writeRobot(dataWriter, "odometry", posWithError, rotWithError);
        writeBoth([](TWriter &writer) { writer.EndObject(); }); // measurement elem
    }
    writeBoth([](TWriter &writer) {
        writer.EndArray(); // measurements
        writer.EndObject(); // data
        writer.EndObject();
    });

    dataOs << dataStringBuffer.GetString() << std::endl;
    groundTruthOs << truthStringBuffer.GetString() << std::endl;
}
