#include "seahowl/core/blade.h"

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/core/utils.h>

#include <chrono/fea/ChMesh.h>
#include <chrono/physics/ChSystemSMC.h>

#include <memory>

seahowl::core::Blade::Blade() {
    m_elasto = std::make_shared<seahowl::elasto::BladeElasto>();
    m_aero = std::make_shared<seahowl::aero::BladeAero>();
}

seahowl::core::Blade::~Blade() {}

void seahowl::core::Blade::build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh) {
    // push reference points
    m_elasto->reference_points.clear();
    m_aero->reference_points.clear();
    for (auto& pt : m_reference_points) {
        m_elasto->reference_points.push_back(seahowl::elasto::BladeReferencePointElasto(pt));
        m_aero->reference_points.push_back(seahowl::aero::BladeReferencePointAero(pt));
    }
    // build aero & elasto
    m_elasto->build(system, mesh);
    m_aero->build();

    // mappings
    compute_mapping_aero2elasto();
    compute_mapping_elasto2aero();
}

void seahowl::core::Blade::set_discretization_elasto(std::vector<double> fractions) {
    m_elasto->discretization_fractions = fractions;
};

void seahowl::core::Blade::set_discretization_aero(std::vector<double> fractions) {
    m_aero->discretization_fractions = fractions;
};

void seahowl::core::Blade::compute_mapping_aero2elasto() {
    // get aero element position (center) from which loads will be applied
    std::vector<double> aero_discretization_fractions;
    for (int ii = 0; ii < m_aero->elements.size(); ii++) {
        aero_discretization_fractions.push_back(m_aero->elements[ii].properties.fraction);
    }
    m_mapping_aero2elasto =
        seahowl::core::get_indice_and_positions(aero_discretization_fractions, m_elasto->discretization_fractions);
}

void seahowl::core::Blade::compute_mapping_elasto2aero() {
    m_mapping_elasto2aero =
        seahowl::core::get_indice_and_positions(m_elasto->discretization_fractions, m_aero->discretization_fractions);
}

void seahowl::core::Blade::prestep(double time) {
    // update loads on elasto part
    update_loads_elasto();
}

void seahowl::core::Blade::poststep(double time) {
    // update position of aero points
    update_positions_aero();
}

void seahowl::core::Blade::update_positions_aero() {
    for (int ii = 0; ii < m_aero->elements.size(); ii++) {
        // update position and rotation of aero elements
        int elasto_element_index = m_mapping_aero2elasto[ii].index;
        double eta = m_mapping_aero2elasto[ii].eta;
        m_elasto->evaluate_position_rotation(m_aero->elements[ii].properties.m_coordinates,
                                             m_aero->elements[ii].properties.rotation, elasto_element_index, eta);

        // update velocity of aero elements
        m_aero->elements[ii].properties.velocity =
            0.5 * (m_elasto->elements[elasto_element_index]->GetNodeA()->GetPos_dt() +
                   m_elasto->elements[elasto_element_index]->GetNodeB()->GetPos_dt());

        // update pitch of aero elements
        m_aero->elements[ii].pitch = m_elasto->pitch;
    }
}

void seahowl::core::Blade::update_loads_elasto() {
    m_elasto->reset_loads();
    if (m_aero->loads.size() != m_mapping_aero2elasto.size()) {
        throw std::runtime_error("length of vector of loads and aero to elasto mapping do not match.");
    }
    for (int ii = 0; ii < m_aero->loads.size(); ii++) {
        m_elasto->accumulate_element_load(m_aero->loads[ii], m_mapping_aero2elasto[ii].index,
                                          m_mapping_aero2elasto[ii].eta);
    }
}
