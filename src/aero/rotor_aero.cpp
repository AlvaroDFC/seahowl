#include "rotor_aero.h"

void RotorAero::build(std::vector<std::shared_ptr<BladeAero>> blades) {
    this->blades = blades;
    compute_chords_solidity();
}

void RotorAero::compute_chords_solidity() {
    auto nblades = blades.size();
    for (int ii = 0; ii < blades.size(); ii++) {
        auto& blade = blades[ii];
        for (int jj = 0; jj < blade->elements.size(); jj++) {
            auto& element = blade->elements[jj];
            auto radius = (element.properties.coordinates - hub_position).Length();
            element.swept_annulus = element.length * 2 * M_PI * radius;
            element.chord_solidity = nblades * element.properties.chord / (2 * CH_C_PI * radius);
            // std::cout << element.swept_annulus << " " << element.chord_solidity << std::endl;
        }
    }
}

void RotorAero::compute_wind_loads_bemt(WindModel& wind_model, double time) {
    for (int ii = 0; ii < blades.size(); ii++) {
        blades[ii]->compute_wind_loads_bemt(wind_model, time);
    }
}
