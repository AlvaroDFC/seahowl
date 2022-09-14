#include "seahowl/core/turbine.h"

seahowl::core::Turbine::Turbine() {
    blades.resize(0);
    rotor = Rotor();
    tower = Tower();
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

void seahowl::core::Turbine::prestep(double time) {
    rotor.prestep(time);
    tower.prestep(time);
}
void seahowl::core::Turbine::poststep(double time) {
    rotor.poststep(time);
    tower.poststep(time);
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
