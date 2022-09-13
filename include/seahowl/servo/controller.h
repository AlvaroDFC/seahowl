#pragma once

#include <seahowl/core/turbine.h>

namespace seahowl {

/**@brief Servo controller module */
namespace servo {

/**@brief Base class for controller */
class Controller {
  public:
    Controller(){};
    ~Controller(){};

    virtual double get_torque_elec() = 0;
    virtual double get_collective_pitch() = 0;
};

/**@brief Variable torque controler */
class ControllerVariableTorque : public seahowl::servo::Controller {
  private:
    double torque_elec = 0.0;
    double torque_elec_previous = 0.0;

  public:
    double target_rpm = 0.0;

    ControllerVariableTorque(){};
    ~ControllerVariableTorque(){};

    void prestep(double torque_total, double rpm);
    void poststep();
    virtual double get_torque_elec();
    virtual double get_collective_pitch();
};

}  // namespace servo
}  // namespace seahowl
