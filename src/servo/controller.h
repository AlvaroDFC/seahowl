#ifndef CONTROLLER_H_
#define CONTROLLER_H_

class Controller {
  public:
    Controller(){};
    ~Controller(){};
};

class ControllerVariableTorque : public Controller {
  private:
    double torque_elec_previous = 0.0;

  public:
    double target_rpm = 0.0;

    ControllerVariableTorque(){};
    ~ControllerVariableTorque(){};

    double get_torque_elec(double torque_total, double rpm);
};

#endif  // CONTROLLER_H_
