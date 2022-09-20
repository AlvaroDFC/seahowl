#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/utils.h>

#include <memory>
#include <vector>
#include <numeric>

#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

using seahowl::elasto::TowerElasto;

TowerElasto::TowerElasto() {}

TowerElasto::~TowerElasto() {}

void TowerElasto::assemble(std::shared_ptr<chrono::fea::ChMesh> mesh) {
    for (auto node : nodes) {
        mesh->AddNode(node);
    }
    for (auto element : elements) {
        mesh->AddElement(element);
    }
}

void TowerElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    build_nodes();
    build_elements_tapered_timoshenko();
};

void TowerElasto::build_nodes() {
    nodes.clear();

    const auto nnodes = discretized_points.size();
    for (auto ii = 0; ii < nnodes; ii++) {
        auto discretized_point = discretized_points[ii];
        auto node_pos = discretized_point.coordinates;

        // get node coordinate system
        chrono::ChVector<> node_axis;
        chrono::ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        } else {
            node_axis =
                (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, chrono::VECT_Y);
        }
        auto node_frame = chrono::ChFrame<>(node_pos, node_rotation);

        // make node
        auto node = chrono_types::make_shared<chrono::fea::ChNodeFEAxyzrot>(node_frame);
        // add node to tower nodes vector
        nodes.push_back(node);
    };
}

void TowerElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    // make first section for tapered section
    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
    auto& discretized_point = discretized_points[0];
    // material properties
    section->SetMassPerUnitLength(discretized_point.density);
    // axial
    section->SetAxialRigidity(discretized_point.stiffness_axial);
    section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
    // foreaft
    section->SetZbendingRigidity(discretized_point.stiffness_foreaft);
    // sideside
    section->SetYbendingRigidity(discretized_point.stiffness_sideside);
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamTaperedTimoshenko>();
        // add element to tower elements vector
        elements.push_back(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create tower section
        auto tower_section = chrono_types::make_shared<chrono::fea::ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        element->SetTaperedSection(tower_section);

        // set first section for tapered section
        tower_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<chrono::fea::ChBeamSectionTimoshenkoAdvancedGeneric>();
        tower_section->SetSectionB(section);
        auto discretized_point = discretized_points[ii];
        // material properties
        section->SetMassPerUnitLength(discretized_point.density);
        // axial
        section->SetAxialRigidity(discretized_point.stiffness_axial);
        section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
        // foreaft
        section->SetZbendingRigidity(discretized_point.stiffness_foreaft);
        // sideside
        section->SetYbendingRigidity(discretized_point.stiffness_sideside);
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);
    }
}

void TowerElasto::set_damping_coefficients(double axial, double edge, double flap, double torsion) {
    chrono::fea::DampingCoefficients damping_coefficients;
    damping_coefficients.bx = axial;
    damping_coefficients.by = edge;
    damping_coefficients.bz = flap;
    damping_coefficients.bt = torsion;
    for (int ii = 0; ii < reference_points.size(); ii++) {
        auto reference_point = reference_points[ii];
        reference_point.damping_coefficients = damping_coefficients;
    }
    for (int ii = 0; ii < elements.size(); ii++) {
        auto section = elements[ii]->GetTaperedSection();
        section->GetSectionA()->SetBeamRaleyghDamping(damping_coefficients);
        section->GetSectionB()->SetBeamRaleyghDamping(damping_coefficients);
    }
}

void TowerElasto::evaluate_position_rotation(chrono::ChVector<double>& position,
                                             chrono::ChQuaternion<double>& rotation,
                                             int element_index,
                                             double eta) {
    auto& element = elements[element_index];

    element->EvaluateSectionFrame(eta, position, rotation);
}

void TowerElasto::reset_loads() {
    for (auto node : nodes) {
        node->SetForce({0.0, 0.0, 0.0});
        node->SetTorque({0.0, 0.0, 0.0});
    }
}

void TowerElasto::accumulate_element_load(chrono::ChVector<double> load, int element_index, double eta) {
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) + " does not exist (max " +
                                 std::to_string(elements.size()) + ").");
    }
    auto& element = elements[element_index];
    chrono::ChVector<double> position{0.0, 0.0, 0.0};
    chrono::ChQuaternion<double> rotation{0.0, 0.0, 0.0, 0.0};
    element->EvaluateSectionFrame(eta, position, rotation);

    // load on first node
    double weight0 = 0.5 * abs(eta - 1);
    auto load0 = load * weight0;
    auto node0 = element->GetNodeA();
    node0->SetForce(node0->GetForce() + load0);
    node0->SetTorque(node0->GetTorque() + (position - node0->GetPos()) % load0);

    // load on second node
    double weight1 = 0.5 * abs(eta - 1);
    auto load1 = load * weight1;
    auto node1 = element->GetNodeB();
    node1->SetForce(node1->GetForce() + load1);
    node1->SetTorque(node1->GetTorque() + (position - node1->GetPos()) % load1);
}
