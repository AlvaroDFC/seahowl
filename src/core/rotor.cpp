#include "rotor.h"

Rotor::Rotor() {
    elasto = RotorElasto();
    aero = RotorAero();
}

void Rotor::update_positions_aero() {
    aero.hub_position = elasto.body_hub->GetPos();
    aero.hub_rotation = elasto.body_hub->GetRot();
}

void Rotor::build(ChSystemSMC& system, std::vector<std::shared_ptr<Blade>> blades) {
    this->blades = blades;

    // get elasto and aero blades pointers
    std::vector<std::shared_ptr<BladeElasto>> blades_elasto;
    std::vector<std::shared_ptr<BladeAero>> blades_aero;
    for (int ii = 0; ii < blades.size(); ii++) {
        blades_elasto.push_back(blades[ii]->elasto);
        blades_aero.push_back(blades[ii]->aero);
    }

    // build elasto
    elasto.build(system, blades_elasto);
    // build aero
    aero.build(blades_aero);
    update_positions_aero();
    aero.compute_chords_solidity();
}

void Rotor::prestep(double time) {
    update_positions_aero();
    aero.compute_chords_solidity();
}
