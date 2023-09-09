#pragma once

#include <functional>
#include <random>
#include <utility>
#include "Util/Math.h"

class TQualityModel {
private:
    // accepts a random number from the half-interval [0, 1), returns quality for point
    using TQualityFunc = std::function<float(float)>;
    TQualityFunc QualityFunc = [](float) { return 1; };

    // indicates how much quality contributes to variance
    float Coef = 1;

    mutable std::uniform_real_distribution<float> distribution{0, 1}; // [0, 1)

protected:
    // use inside ErrorModel
    std::pair<float, float> GetQualityCoef() const {
        float quality = QualityFunc(distribution(GetRandGen()));
        return {quality, 1 + (1 - quality) * Coef};
    }

public:
    void SetQualityFunction(TQualityFunc func) {
        QualityFunc = std::move(func);
    }

    void SetQualityCoef(float coef) {
        Coef = coef;
    }
};