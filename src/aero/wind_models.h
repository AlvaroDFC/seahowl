#ifndef WIND_MODELS_H_
#define WIND_MODELS_H_

#include "chrono/core/ChVector.h"

class WindModel {
  public:
    WindModel() {}
    ~WindModel() {}
    virtual ChVector<double> get_wind_velocity(ChVector<double>& position, double time) = 0;
};

class ConstantWind : public WindModel {
  public:
    double density;
    ChVector<double> wind_velocity;

    ConstantWind() {
        wind_velocity = ChVector<double>(0.0, 0.0, 0.0);
        density = 1.225;
    }

    ~ConstantWind() {}

    void set_wind_velocity(ChVector<double> velocity) { wind_velocity = velocity; }
    ChVector<double> get_wind_velocity(ChVector<double>& position, double time) { return wind_velocity; }
};

#endif  // WIND_MODELS_H_
