#pragma once

#include <seahowl/core/turbine.h>

namespace seahowl {
namespace core {
class Turbine;
}
/**@brief Servo controller module */
namespace servo {

/**@brief Base class for controller */
class Controller {
  public:
    Controller(){};
    ~Controller(){};

    virtual void init(double time, double dt, seahowl::core::Turbine& turbine){};
    virtual void step(double time, double dt, seahowl::core::Turbine& turbine){};
    virtual void poststep(double time, double dt, seahowl::core::Turbine& turbine){};
    virtual double get_torque_elec() { return 0.0; };
    virtual double get_collective_pitch() { return 0.0; };
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

    void step(double torque_total, double rpm);
    virtual void step(double time, double dt, seahowl::core::Turbine& turbine) override;
    virtual void poststep(double time, double dt, seahowl::core::Turbine& turbine) override;
    void poststep();
    virtual double get_torque_elec() override;
    virtual double get_collective_pitch() override;
};

}  // namespace servo
}  // namespace seahowl
