#pragma once

#include "mutil/mutil.h"
#include "Util/Json.h"

class IErrorModel {
public:
    [[nodiscard]] virtual mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const = 0;

    [[nodiscard]] virtual mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const = 0;

    [[nodiscard]] virtual TLidarPoint AddLidarError(const mutil::Vector3 &point) const = 0;

    // TODO: write quality's stuff
    virtual void Write(TWriter &writer, const char *key) const = 0;

    virtual ~IErrorModel() = default;
};
