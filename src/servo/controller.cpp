#include "seahowl/servo/controller.h"

#include <cmath>

void seahowl::servo::ControllerVariableTorque::prestep(double torque_total, double rpm) {
    // total_torque includes aero torque + previous elec torque
    double torque_aero = torque_total + torque_elec_previous;
    double torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
    if (rpm / target_rpm < 0.0) {
        torque_elec = 0.0;
    }
}

void seahowl::servo::ControllerVariableTorque::poststep() {
    torque_elec_previous = torque_elec;
}

double seahowl::servo::ControllerVariableTorque::get_torque_elec() {
    return torque_elec;
}

double seahowl::servo::ControllerVariableTorque::get_collective_pitch() {
    return 0.0;
}
