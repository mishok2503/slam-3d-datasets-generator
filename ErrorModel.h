#pragma once

#include "mutil/mutil.h"
#include "util.h"

class IErrorModel {
public:
    virtual mutil::Vector3 AddPositionError(const mutil::Vector3 &posDelta) const = 0;

    virtual mutil::Vector3 AddRotationError(const mutil::Vector3 &rotDelta) const = 0;

    virtual mutil::Vector3 AddLidarError(const mutil::Vector3 &point) const = 0;

    virtual void Write(TWriter &writer, const char *key) const = 0;

    virtual ~IErrorModel() = default;
};
