#include "seahowl/core/turbine.h"

#include <chrono/physics/ChBody.h>

seahowl::core::Turbine::Turbine() {
    blades.resize(0);
    rotor = Rotor();
    tower = Tower();
    controller = std::make_shared<seahowl::servo::Controller>();
}

seahowl::core::Turbine::~Turbine() {}

void seahowl::core::Turbine::assemble(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh) {
    // assemble blades
    for (auto& blade : blades) {
        blade->assemble(system, mesh);
    }

    // assemble rotor & tower
    rotor.assemble(system);
    tower.assemble(mesh);

    // link tower to rotor
    rotor.elasto.link_tower(tower.elasto, system);
}

void seahowl::core::Turbine::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    // build rotor & tower
    rotor.build(blades);
    tower.build();
}

void seahowl::core::Turbine::init(double time, double dt) {
    rotor.prestep(time, dt);
    tower.prestep(time, dt);
    rotor.poststep(time, dt);
    tower.poststep(time, dt);

    controller->init(time, dt, *this);
}

void seahowl::core::Turbine::prestep(double time, double dt) {
    rotor.prestep(time, dt);
    tower.prestep(time, dt);
}
void seahowl::core::Turbine::poststep(double time, double dt) {
    // controller step
    controller->step(time, dt, *this);
    // apply torque from comtroller
    if (controller->has_torque_control) {
        auto torque_elec = controller->get_torque_elec();
        // apply torque elec to hub rigid body
        rotor.elasto.body_hub->Empty_forces_accumulators();
        // torque elec is apply on Z axis of hub body (locally)
        rotor.elasto.body_hub->Accumulate_torque(chrono::ChVector<double>(0.0, 0.0, -torque_elec), true);
    }
    // apply pitch from controller
    if (controller->has_pitch_control) {
        auto collective_pitch_increment = controller->get_collective_pitch() - rotor.elasto.pitch_collective;
        rotor.elasto.apply_collective_pitch_increment(collective_pitch_increment);
    }

    // poststeps
    rotor.poststep(time, dt);
    tower.poststep(time, dt);

    // controller poststep
    controller->poststep(time, dt, *this);
}

void seahowl::core::Turbine::translate(chrono::ChVector<double> translation_vector) {
    rotor.elasto.translate(translation_vector);
    tower.elasto.translate(translation_vector);
}

void seahowl::core::Turbine::rotate(double angle, chrono::ChVector<double> axis) {
    rotor.elasto.rotate(angle, axis);
    tower.elasto.rotate(angle, axis);
}

void seahowl::core::Turbine::compute_wind_loads(seahowl::aero::WindModel& wind_model, double time) {
    rotor.aero.compute_wind_loads_bemt(wind_model, time);
    tower.aero.compute_wind_loads_morison(wind_model, time);
}

double seahowl::core::Turbine::get_generated_power() {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rot_rads = rotor.elasto.get_rpm() * (2.0 * chrono::CH_C_PI / 60.0) * gearbox_ratio * gearbox_efficiency;
    // get torque elec from rotor
    auto torque_elec = controller->get_torque_elec();

    // calculate power
    auto power = torque_elec * rot_rads * generator_efficiency;
    return power;
}

double seahowl::core::Turbine::get_generator_rpm() {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rpm = rotor.elasto.get_rpm() * gearbox_ratio * gearbox_efficiency;
    return rpm;
}
