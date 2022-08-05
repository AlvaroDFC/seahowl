#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/utils.h>

#include <memory>
#include <vector>

#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

using seahowl::elasto::TowerElasto;

void TowerElasto::build(std::shared_ptr<chrono::fea::ChMesh> mesh) {
    discretized_points = get_discretized_points(discretization_fractions, reference_points);
    build_nodes(mesh);
    build_elements_tapered_timoshenko(mesh);
};

void TowerElasto::build_nodes(std::shared_ptr<chrono::fea::ChMesh> mesh) {
    nodes.clear();

    const auto nnodes = discretized_points.size();
    for (auto  ii = 0; ii < nnodes; ii++) {
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
        // add node to mesh
        mesh->AddNode(node);
    };
}

void TowerElasto::build_elements_tapered_timoshenko(std::shared_ptr<chrono::fea::ChMesh> mesh) {
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
        // add element to mesh
        mesh->AddElement(element);
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

void TowerElasto::rotate(double angle, chrono::ChVector<double> axis) {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}

void TowerElasto::translate(chrono::ChVector<double> translation_vector) {
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        node->SetPos(node->GetPos() + translation_vector);
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

double TowerElasto::get_mass() {
    double total_mass = 0.0;
    for (int ii = 0; ii < elements.size(); ii++) {
        total_mass += elements[ii]->GetMass();
    }
    return total_mass;
}
