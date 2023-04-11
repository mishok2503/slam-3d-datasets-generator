#include "Simulator.h"

#include "util.h"
#include "Robot.h"

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

    constexpr int SEED = 239;
    std::mt19937 randomGenerator{SEED};

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
            auto pointWithError = ErrorModel->AddLidarError(point.data);
            writeVector3(truthWriter, "coordinates", point.data);
            writeVector3(dataWriter, "coordinates", pointWithError);
            writeBoth([](TWriter &writer) { writer.EndObject(); }); // lidar_data elem
        }
        writeBoth([](TWriter &writer) { writer.EndArray(); }); // lidar_data
        auto const&[pos, rot] = Robot.Move(Map);
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
