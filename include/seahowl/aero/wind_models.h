#pragma once

#include "chrono/core/ChVector.h"

/**@brief Base class for wind models */
class WindModel {
  public:
    double density = 1.225;

    WindModel() {}
    ~WindModel() {}

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) = 0;

    double get_density() { return density; }
};

/**@brief Constant wind models */
class ConstantWind : public WindModel {
  public:
    double density;
    chrono::ChVector<double> wind_velocity;

    ConstantWind() { wind_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0); }

    ~ConstantWind() {}

    void set_wind_velocity(chrono::ChVector<double> velocity) { wind_velocity = velocity; }
    chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) { return wind_velocity; }
};

