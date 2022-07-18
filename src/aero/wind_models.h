#ifndef WIND_MODELS_H_
#define WIND_MODELS_H_

#include "chrono/core/ChVector.h"

class WindModel {
  public:
    WindModel() {}
    ~WindModel() {}
    virtual ChVector<double> get_wind_speed(ChVector<double>& position, double time) = 0;
};

class ConstantWind : public WindModel {
  public:
    double density;
    ChVector<double> wind_speed;

    ConstantWind() {
        wind_speed = ChVector<double>(0.0, 0.0, 0.0);
        density = 1.225;
    }

    ~ConstantWind() {}

    void set_wind_speed(ChVector<double> speed) { wind_speed = speed; }
    ChVector<double> get_wind_speed(ChVector<double>& position, double time) { return wind_speed; }
};

#endif  // WIND_MODELS_H_
