#include "turbine.h"

Turbine::Turbine() {
    rotor = RotorElasto();
    tower = TowerElasto();
}

void Turbine::build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
    // build blades
    std::vector<std::shared_ptr<BladeElasto>> blades_elasto;
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->build(system, mesh);
        blades_elasto.push_back(blades[ii]->elasto);
    }
    // build rotor
    // TODO: remove BladeElasto dependency by making a Rotor core that takes in vector of Blade
    rotor.build(system, blades_elasto);
    // build tower
    tower.build(mesh);
    // link tower to rotor
    rotor.link_tower(tower, system);
}

void Turbine::prestep(double time, WindModel& wind_model) {
    for (int ii = 0; ii < blades.size(); ii++) {
        // rotor[ii]->prestep(time);
        blades[ii]->prestep(time, wind_model);
    }
}

void Turbine::translate(ChVector<double> translation_vector) {
    rotor.translate(translation_vector);
    tower.translate(translation_vector);
}

void Turbine::rotate(double angle, ChVector<double> axis) {
    rotor.rotate(angle, axis);
    tower.rotate(angle, axis);
}
