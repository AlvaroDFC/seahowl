#include "seahowl/servo/controller.h"

#include <seahowl/servo/controller_discon.h>

#include <cmath>

double seahowl::servo::ControllerVariableTorque::get_torque_elec(double torque_total, double rpm) {
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



double seahowl::servo::ControllerDISCON::get_torque_elec(double Omega, double time, double dt) {

    pImpl.SetAvrSWAP(21,Omega);
    pImpl.SetAvrSWAP(2,time);
    pImpl.SetAvrSWAP(3,dt);

    pImpl.Call();
    double torque_elec = pImpl.GetAvrSWAP(47);
    
    return torque_elec;

}


seahowl::servo::ControllerDISCON::ControllerDISCON(std::string infile, std::string outname){
    pImpl.Init(infile, outname);
}