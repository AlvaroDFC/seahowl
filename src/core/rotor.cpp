#include "seahowl/core/rotor.h"

#include <chrono/physics/ChBody.h>

#include <memory>
#include <vector>

using seahowl::core::Rotor;

Rotor::Rotor() {
    elasto = seahowl::elasto::RotorElasto();
    aero = seahowl::aero::RotorAero();
}

Rotor::~Rotor() {}

void Rotor::update_positions_aero() {
    aero.hub_position = elasto.body_hub->GetPos();
    aero.hub_rotation = elasto.body_hub->GetRot();
}

void Rotor::build(chrono::ChSystemSMC& system, std::vector<std::shared_ptr<seahowl::core::Blade>> blades) {
    this->blades = blades;

    // get elasto and aero blades pointers
    std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades_elasto;
    std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades_aero;
    for (auto& blade : blades) {
        blades_elasto.push_back(blade->elasto);
        blades_aero.push_back(blade->aero);
        blade->update_positions_aero();
    }

    // build elasto
    elasto.build(system, blades_elasto);
    // update blade aero positions from new elasto positions
    for (auto& blade : blades) {
        blade->update_positions_aero();
    }
    // update hub position from elasto
    update_positions_aero();
    // build aero
    aero.build(blades_aero);
}

void Rotor::prestep(double time) {
    for (auto& blade : blades) {
        blade->prestep(time);
    }
}

void Rotor::poststep(double time) {
    for (auto& blade : blades) {
        blade->poststep(time);
    }
    update_positions_aero();
    aero.compute_chords_solidity();
}
