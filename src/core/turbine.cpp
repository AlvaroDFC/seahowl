#include "seahowl/core/turbine.h"

Turbine::Turbine() {
    rotor = Rotor();
    tower = TowerElasto();
}

void Turbine::build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
    // build blades
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->build(system, mesh);
    }
    // build rotor
    rotor.build(system, blades);
    // build tower
    tower.build(mesh);
    // link tower to rotor
    rotor.elasto.link_tower(tower, system);
}

void Turbine::prestep(double time) {
    rotor.prestep(time);
}
void Turbine::poststep(double time) {
    rotor.poststep(time);
}

void Turbine::translate(ChVector<double> translation_vector) {
    rotor.elasto.translate(translation_vector);
    tower.translate(translation_vector);
}

void Turbine::rotate(double angle, ChVector<double> axis) {
    rotor.elasto.rotate(angle, axis);
    tower.rotate(angle, axis);
}

void Turbine::compute_wind_loads(WindModel& wind_model, double time) {
    rotor.aero.compute_wind_loads_bemt(wind_model, time);
}
