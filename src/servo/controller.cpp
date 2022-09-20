#include "seahowl/servo/controller.h"

#include <cmath>

using namespace seahowl::servo;

Controller::Controller() {
    has_pitch_control = false;
    has_torque_control = false;
}

Controller::~Controller() {}

void Controller::init(double time, double dt, seahowl::core::Turbine& turbine) {}

void Controller::step(double time, double dt, seahowl::core::Turbine& turbine) {}

void Controller::poststep(double time, double dt, seahowl::core::Turbine& turbine) {}

double Controller::get_torque_elec() {
    return 0.0;
}

double Controller::get_collective_pitch() {
    return 0.0;
}

ControllerVariableTorque::ControllerVariableTorque() {
    has_pitch_control = false;
    has_torque_control = true;
}

ControllerVariableTorque::~ControllerVariableTorque() {}

void ControllerVariableTorque::step(double torque_total, double rpm) {
    // total_torque includes aero torque + previous elec torque
    double torque_aero = torque_total + torque_elec_previous;
    double torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
    if (rpm / target_rpm < 0.0) {
        torque_elec = 0.0;
    }
}

void ControllerVariableTorque::poststep() {
    torque_elec_previous = torque_elec;
}

void ControllerVariableTorque::poststep(double time, double dt, seahowl::core::Turbine& turbine) {
    poststep();
}

void ControllerVariableTorque::step(double time, double dt, seahowl::core::Turbine& turbine) {
    double rpm = turbine.rotor.elasto.get_rpm();
    double torque_total = turbine.rotor.elasto.get_torque();
    // total_torque includes aero torque + previous elec torque
    double torque_aero = torque_total + torque_elec_previous;
    double torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
    if (rpm / target_rpm < 0.0) {
        torque_elec = 0.0;
    }
}

double ControllerVariableTorque::get_torque_elec() {
    return torque_elec;
}
