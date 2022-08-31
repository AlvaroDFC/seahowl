#include <seahowl/core/tower.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Tower::Tower() {
    elasto = TowerElasto();
    aero = TowerAero();
}

Tower::~Tower() {}

void Tower::build(std::shared_ptr<chrono::fea::ChMesh> mesh) {
    // push reference points
    elasto.reference_points.clear();
    aero.reference_points.clear();
    for (auto& pt : reference_points) {
        elasto.reference_points.push_back(TowerReferencePointElasto(pt));
        aero.reference_points.push_back(TowerReferencePointAero(pt));
    }
    // build
    elasto.build(mesh);
};

void Tower::set_discretization_elasto(std::vector<double> fractions) {
    elasto.discretization_fractions = fractions;
};

void Tower::set_discretization_aero(std::vector<double> fractions) {
    aero.discretization_fractions = fractions;
};

void Tower::prestep(double time) {}

void Tower::poststep(double time) {}
