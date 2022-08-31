#include "seahowl/core/turbine.h"

seahowl::core::Turbine::Turbine() {
    m_blades.resize(0);
    m_rotor = Rotor();
    m_tower = Tower();
}

seahowl::core::Turbine::~Turbine() {}

void seahowl::core::Turbine::build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh) {
    // build blades
    for (auto& blade : m_blades) {
        blade->build(system, mesh);
    }

    // build rotor & tower
    m_rotor.build(system, m_blades);
    m_tower.build(mesh);

    // link tower to rotor
    m_rotor.elasto.link_tower(m_tower.elasto, system);
}

void seahowl::core::Turbine::prestep(double time) {
    m_rotor.prestep(time);
}
void seahowl::core::Turbine::poststep(double time) {
    m_rotor.poststep(time);
}

void seahowl::core::Turbine::translate(chrono::ChVector<double> translation_vector) {
    m_rotor.elasto.translate(translation_vector);
    m_tower.elasto.translate(translation_vector);
}

void seahowl::core::Turbine::rotate(double angle, chrono::ChVector<double> axis) {
    m_rotor.elasto.rotate(angle, axis);
    m_tower.elasto.rotate(angle, axis);
}

void seahowl::core::Turbine::compute_wind_loads(seahowl::aero::WindModel& wind_model, double time) {
    m_rotor.aero.compute_wind_loads_bemt(wind_model, time);
}
