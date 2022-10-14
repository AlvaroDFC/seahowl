#pragma once

#include <chrono/core/ChVector.h>

namespace seahowl {
namespace aero {

/**@brief Base class for wind models

@todo Move in another module. Environment ?
*/
class WindModel {
  public:
    double density = 1.225;  ///< Air density

    WindModel();
    ~WindModel();

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) const;
    double get_density() const;
};

/**@brief Constant wind models */
class ConstantWind : public WindModel {
  public:
    chrono::ChVector<double> wind_velocity;
    double shear_coefficient = 0.0;
    double reference_height = 150.0;
    double reference_length = 240.0;
    chrono::ChVector<double> direction_gravity;

    ConstantWind();
    ~ConstantWind();

    void set_wind_velocity(chrono::ChVector<double> velocity);
    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) const;
};

/**@brief Wind ramp model */
class WindRamp : public WindModel {
  public:
    double time_start = 0.0;
    double time_stop = 1.0;
    chrono::ChVector<double> wind_velocity_start;
    chrono::ChVector<double> wind_velocity_stop;
    double shear_coefficient = 0.0;
    double reference_height = 150.0;
    double reference_length = 240.0;
    chrono::ChVector<double> direction_gravity;

    WindRamp();
    ~WindRamp();

    void set_wind_velocity_start(chrono::ChVector<double> velocity);
    void set_wind_velocity_stop(chrono::ChVector<double> velocity);
    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) const;
};

}  // namespace aero
}  // namespace seahowl