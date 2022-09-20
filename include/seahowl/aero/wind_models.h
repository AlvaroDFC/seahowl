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

    WindModel() {}
    ~WindModel() {}

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) {
        return chrono::ChVector<double>(0.0, 0.0, 0.0);
    };

    double get_density() { return density; }
};

/**@brief Constant wind models */
class ConstantWind : public WindModel {
  public:
    chrono::ChVector<double> wind_velocity;

    ConstantWind() { wind_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0); }

    ~ConstantWind() {}

    void set_wind_velocity(chrono::ChVector<double> velocity) { wind_velocity = velocity; }

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) override {
        return wind_velocity;
    }
};

}  // namespace aero
}  // namespace seahowl