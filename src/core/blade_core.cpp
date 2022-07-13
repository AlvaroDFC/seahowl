#include "blade_core.h"

Blade::Blade() {
    elasto = std::make_shared<BladeElasto>();
    aero = std::make_shared<BladeAero>();
}

void Blade::build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
    // push reference points
    elasto->reference_points.clear();
    /* aero->reference_points.clear(); */
    for (int ii = 0; ii < reference_points.size(); ii++) {
        elasto->reference_points.push_back(BladeReferencePointElasto(reference_points[ii]));
        /* aero->reference_points.push_back(BladeReferencePointAero(reference_points[ii])); */
    }
    // build elasto
    elasto->build(system, mesh);

    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
}

void Blade::set_discretization_elasto(std::vector<double> fractions) {
    elasto->discretization_fractions = fractions;
};

void Blade::set_discretization_aero(std::vector<double> fractions) {
    aero->discretization_fractions = fractions;
};

void Blade::compute_mapping_aero2elasto() {
    mapping_aero2elasto = get_indice_and_positions(aero->discretization_fractions, elasto->discretization_fractions);
}

void Blade::compute_mapping_elasto2aero() {
    mapping_elasto2aero = get_indice_and_positions(elasto->discretization_fractions, aero->discretization_fractions);
}

void Blade::prestep() {
    // reset loads on elasto part
    elasto->reset_loads();
    // update position of aero points
    update_positions_aero();
    // compute loads from aero
    aero->compute_loads();
    // update loads on elasto part
    update_loads_elasto();
}

void Blade::update_positions_aero() {
    for (int ii = 0; ii < aero->discretized_points.size(); ii++) {
        int element_index = mapping_aero2elasto[ii].index;
        double eta = mapping_aero2elasto[ii].eta;
        elasto->evaluate_position_rotation(aero->discretized_points[ii].coordinates,
                                           aero->discretized_points[ii].rotation, element_index, eta);
    }
}

void Blade::update_loads_elasto() {
    elasto->reset_loads();
    if (aero->loads.size() != mapping_aero2elasto.size()) {
        std::runtime_error("length of vector of loads and aero to elasto mapping do not match.");
    }
    for (int ii = 0; ii < aero->loads.size(); ii++) {
        elasto->accumulate_element_load(aero->loads[ii], mapping_aero2elasto[ii].index, mapping_aero2elasto[ii].eta);
    }
}
