#include "seahowl/servo/controller.h"

#include <cmath>

double ControllerVariableTorque::get_torque_elec(double torque_total, double rpm) {
    // total_torque includes aero torque + previous elec torque
    double torque_aero = torque_total + torque_elec_previous;
    double torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
    if (rpm / target_rpm < 0.0) {
        torque_elec = 0.0;
    }
    ///@todo: add prestep/poststep functions to store torque_elec_previous
    ///       because currently calling this function twice in a row in the
    ///       same step might cause problems
    torque_elec_previous = torque_elec;
    return torque_elec;
}
