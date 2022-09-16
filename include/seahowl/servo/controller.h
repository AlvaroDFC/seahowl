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
    bool has_pitch_control;
    bool has_torque_control;

    Controller();
    ~Controller();

    virtual void init(double time, double dt, seahowl::core::Turbine& turbine);
    virtual void step(double time, double dt, seahowl::core::Turbine& turbine);
    virtual void poststep(double time, double dt, seahowl::core::Turbine& turbine);
    virtual double get_torque_elec();
    virtual double get_collective_pitch();
};

/**@brief Variable torque controler */
class ControllerVariableTorque : public seahowl::servo::Controller {
  private:
    double torque_elec = 0.0;
    double torque_elec_previous = 0.0;

  public:
    double target_rpm = 0.0;

    ControllerVariableTorque();
    ~ControllerVariableTorque();

    void step(double torque_total, double rpm);
    virtual void step(double time, double dt, seahowl::core::Turbine& turbine) override;
    virtual void poststep(double time, double dt, seahowl::core::Turbine& turbine) override;
    void poststep();
    virtual double get_torque_elec() override;
};

}  // namespace servo
}  // namespace seahowl
