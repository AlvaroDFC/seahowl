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
    double shear_coefficient = 0.0;
    double reference_height = 150.0;
    double reference_length = 240.0;
    chrono::ChVector<double> direction_gravity;

    ConstantWind() {
        wind_velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
        direction_gravity = chrono::ChVector<double>(0.0, 0.0, -1.0);
    }

    ~ConstantWind() {}

    void set_wind_velocity(chrono::ChVector<double> velocity) { wind_velocity = velocity; }

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) override {
        double distance = position ^ (-direction_gravity);
        if (distance > reference_length) {
            distance = reference_length;
        }
        return wind_velocity * pow(distance / reference_height, shear_coefficient);
    }
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

    WindRamp() {
        wind_velocity_start = chrono::ChVector<double>(0.0, 0.0, 0.0);
        wind_velocity_stop = chrono::ChVector<double>(0.0, 0.0, 0.0);
        direction_gravity = chrono::ChVector<double>(0.0, 0.0, -1.0);
    }

    ~WindRamp() {}

    void set_wind_velocity_start(chrono::ChVector<double> velocity) { wind_velocity_start = velocity; }
    void set_wind_velocity_stop(chrono::ChVector<double> velocity) { wind_velocity_stop = velocity; }

    virtual chrono::ChVector<double> get_wind_velocity(chrono::ChVector<double>& position, double time) override {
        auto velocity = wind_velocity_start;
        if (time >= time_start) {
            double w1 = 1.0 - std::min((time - time_start) / time_stop, 1.0);
            double w2 = 1.0 - w1;
            velocity = w1 * wind_velocity_start + w2 * wind_velocity_stop;
        }
        double distance = position ^ (-direction_gravity);
        if (distance > reference_length) {
            distance = reference_length;
        }
        velocity *= pow(distance / reference_height, shear_coefficient);
        return velocity;
    }
};

}  // namespace aero
}  // namespace seahowl