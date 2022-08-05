#pragma once

namespace seahowl {

/**@brief Servo controller module */
namespace servo {

/**@brief Base class for controller */
class Controller {
  public:
    Controller(){};
    ~Controller(){};
};

/**@brief Variable torque controler */
class ControllerVariableTorque : public Controller {
  private:
    double torque_elec_previous = 0.0;

  public:
    double target_rpm = 0.0;

    ControllerVariableTorque(){};
    ~ControllerVariableTorque(){};

    double get_torque_elec(double torque_total, double rpm);
};

}  // namespace servo
}  // namespace seahowl
