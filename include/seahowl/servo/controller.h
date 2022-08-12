#pragma once
#include <seahowl/servo/controller_discon.h>

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


/**@brief DISCON controler */
class ControllerDISCON : public Controller {
  private:
    double torque_elec_previous = 0.0;

  public:
    seahowl::servo::DisconController pImpl;
    double target_rpm = 0.0;

    ControllerDISCON(std::string infile=u8"DISCON.IN", std::string outname=u8"simDEBUG.RO.dbg");
    ~ControllerDISCON(){};

    double get_torque_elec(double Omega, double time, double dt);
};




}  // namespace servo
}  // namespace seahowl
