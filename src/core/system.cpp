#include <seahowl/core/system.h>

using namespace seahowl::core;

System::System(Turbine turbine, seahowl::aero::WindModel& wind_model) : turbine(turbine), wind_model(wind_model) {}

System::~System() {}

void System::init(double time, double dt) {}

void System::prestep(double time, double dt) {
    // compute forces on rotor and tower
    turbine.rotor.aero.compute_wind_loads_bemt(wind_model, time);
    turbine.tower.aero.compute_wind_loads_morison(wind_model, time);

    // turbine prestep (accumulates loads from aero to elasto)
    turbine.prestep(time);
}

void System::poststep(double time, double dt) {
    // turbine poststep
    turbine.poststep(time);
}

SystemVariableTorque::SystemVariableTorque(Turbine turbine,
                                           seahowl::aero::WindModel& wind_model,
                                           seahowl::servo::ControllerVariableTorque controller)
    : System(turbine, wind_model), controller(controller) {}

SystemVariableTorque::~SystemVariableTorque() {}

void SystemVariableTorque::init(double time, double dt) {}

void SystemVariableTorque::prestep(double time, double dt) {
    // turbine prestep
    turbine.prestep(time);

    // controller prestep
    if (controller.target_rpm > 0.0) {
        // get torque elec from controller
        double rpm = turbine.rotor.elasto.get_rpm();
        double torque_total = turbine.rotor.elasto.get_torque();
        controller.step(torque_total, rpm);
        auto torque_elec = controller.get_torque_elec();
        // apply torque elec to hub rigid body
        turbine.rotor.elasto.body_hub->Empty_forces_accumulators();
        // torque elec is apply on Z axis of hub body (locally)
        turbine.rotor.elasto.body_hub->Accumulate_torque(chrono::ChVector<double>(0.0, 0.0, torque_elec), true);
    }
}
void SystemVariableTorque::poststep(double time, double dt) {
    controller.poststep();
    // turbine poststep
    turbine.poststep(time);

    // controller poststep
    controller.poststep();
}

SystemDISCON::SystemDISCON(Turbine turbine,
                           seahowl::aero::WindModel& wind_model,
                           seahowl::servo::ControllerDISCON controller)
    : System(turbine, wind_model), controller(controller) {}

SystemDISCON::~SystemDISCON() {}

void SystemDISCON::init(double time, double dt) {
    turbine.prestep(time);
    turbine.poststep(time);
    // initialize controller
    controller.init(time, dt, turbine);
}

void SystemDISCON::poststep(double time, double dt) {
    // controller step
    controller.step(time, dt, turbine);
    auto torque_elec = controller.get_torque_elec();
    auto collective_pitch_increment = controller.get_collective_pitch() - turbine.rotor.elasto.pitch_collective;
    // apply torque elec to hub rigid body
    turbine.rotor.elasto.body_hub->Empty_forces_accumulators();
    // torque elec is apply on Z axis of hub body (locally)
    turbine.rotor.elasto.body_hub->Accumulate_torque(chrono::ChVector<double>(0.0, 0.0, torque_elec), true);
    // apply pitch from controller
    turbine.rotor.elasto.apply_collective_pitch_increment(collective_pitch_increment);

    // turbine poststep
    turbine.poststep(time);

    // controller poststep
    controller.poststep(time, dt, turbine);
}