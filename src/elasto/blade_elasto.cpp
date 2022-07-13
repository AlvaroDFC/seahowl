#include "blade_elasto.h"

#include "chrono/fea/ChBuilderBeam.h"

BladeElasto::BladeElasto() {}

void BladeElasto::build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
    // blade
    //
    discretized_points = get_discretized_points(discretization_fractions, reference_points);
    build_nodes(mesh);
    if (fpm_mode) {
        build_elements_tapered_timoshenko_fpm(mesh);
    } else {
        build_elements_tapered_timoshenko(mesh);
    }
    build_loads(system);
};

void BladeElasto::build_nodes(std::shared_ptr<ChMesh> mesh) {
    nodes.clear();
    int nnodes = discretized_points.size();
    for (int ii = 0; ii < nnodes; ii++) {
        auto discretized_point = discretized_points[ii];
        auto node_pos = discretized_point.coordinates;

        // get node coordinate system
        ChVector<> node_axis;
        ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        } else {
            node_axis =
                (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        }
        // apply structural twist
        ChMatrix33<> twist_matrix(Q_from_AngAxis(discretized_point.structural_twist, node_axis));
        node_rotation = twist_matrix * node_rotation;
        auto node_frame = ChFrame<>(node_pos, node_rotation);

        // make node
        auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(node_frame);
        // add node to blade nodes vector
        nodes.push_back(node);
        // add node to mesh
        mesh->AddNode(node);
    };
};

void BladeElasto::build_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh) {
    elements.clear();
    int nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    // make first section for tapered section
    auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();
    auto discretized_point = discretized_points[0];
    // offsets
    section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
    section->SetCentroidY(discretized_point.offset_elastic.y());
    section->SetCentroidZ(-discretized_point.offset_elastic.x());
    // material properties
    section->SetMassPerUnitLength(discretized_point.mass_matrix(0, 0));
    // axial
    section->SetAxialRigidity(discretized_point.stiffness_matrix(0, 0));
    section->SetXtorsionRigidity(discretized_point.stiffness_matrix(3, 3));
    // flap
    section->SetYbendingRigidity(discretized_point.stiffness_matrix(4, 4));
    // edge
    section->SetZbendingRigidity(discretized_point.stiffness_matrix(5, 5));
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (int ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<ChElementBeamTaperedTimoshenko>();
        // add element to blade elements vector
        elements.push_back(element);
        // add element to mesh
        mesh->AddElement(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create blade section
        auto blade_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        element->SetTaperedSection(blade_section);

        // set first section for tapered section
        blade_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();
        blade_section->SetSectionB(section);
        auto discretized_point = discretized_points[ii];
        // offsets
        section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
        section->SetCentroidY(discretized_point.offset_elastic.y());
        section->SetCentroidZ(-discretized_point.offset_elastic.x());
        // material properties
        section->SetMassPerUnitLength(discretized_point.mass_matrix(0, 0));
        // axial
        section->SetAxialRigidity(discretized_point.stiffness_matrix(0, 0));
        section->SetXtorsionRigidity(discretized_point.stiffness_matrix(3, 3));
        // flap
        section->SetYbendingRigidity(discretized_point.stiffness_matrix(4, 4));
        // edge
        section->SetZbendingRigidity(discretized_point.stiffness_matrix(5, 5));
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->GetRot() * nodes[ii - 1]->GetRot().GetInverse()).GetNormalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative =
            ChQuaternion<>(rotation_relative[0], rotation_relative[3], rotation_relative[2], rotation_relative[1]);
        element->SetNodeBreferenceRot(rotation_relative);
    }
}
void BladeElasto::build_elements_tapered_timoshenko_fpm(std::shared_ptr<ChMesh> mesh) {
    elements.clear();
    int nelements = nodes.size() - 1;

    ChMatrixNM<double, 6, 6> mm;
    for (int jj = 0; jj < 6; jj++) {
        mm(jj, jj) = 1.0;
    }
    // make first section for tapered section
    auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();
    auto discretized_point = discretized_points[0];
    // offsets
    section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
    section->SetCentroidY(discretized_point.offset_elastic.y());
    section->SetCentroidZ(-discretized_point.offset_elastic.x());
    // material properties
    section->SetMassMatrixFPM(discretized_point.mass_matrix);
    section->SetStiffnessMatrixFPM(discretized_point.stiffness_matrix);
    // damping
    section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

    for (int ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<ChElementBeamTaperedTimoshenkoFPM>();
        // add element to blade elements vector
        elements.push_back(element);
        // add element to mesh
        mesh->AddElement(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);

        // create blade section
        auto blade_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
        element->SetTaperedSection(blade_section);

        // set first section for tapered section
        blade_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();
        blade_section->SetSectionB(section);
        auto discretized_point = discretized_points[ii];
        // offsets
        section->SetCenterOfMass(discretized_point.offset_gravity.y(), -discretized_point.offset_gravity.x());
        section->SetCentroidY(discretized_point.offset_elastic.y());
        section->SetCentroidZ(-discretized_point.offset_elastic.x());
        // material properties
        section->SetMassMatrixFPM(discretized_point.mass_matrix);
        section->SetStiffnessMatrixFPM(discretized_point.stiffness_matrix);
        // damping
        section->SetBeamRaleyghDamping(discretized_point.damping_coefficients);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->GetRot() * nodes[ii - 1]->GetRot().GetInverse()).GetNormalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative =
            ChQuaternion<>(rotation_relative[0], rotation_relative[3], rotation_relative[2], rotation_relative[1]);
        element->SetNodeBreferenceRot(rotation_relative);
    }
}

void BladeElasto::build_loads(ChSystemSMC& system) {
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);
    for (int ii = 0; ii < elements.size(); ii++) {
        auto element = elements[ii];
        std::shared_ptr<ChLoad<ChLoaderWeighted>> loader_weighted(new ChLoad<ChLoaderWeighted>(element));
        loaders_aero.push_back(loader_weighted);
        loadcontainer->Add(loader_weighted);
    }
}

void BladeElasto::rotate(double angle, ChVector<double> axis) {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}

void BladeElasto::translate(ChVector<double> translation_vector) {
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        node->SetPos(node->GetPos() + translation_vector);
    }
}

void BladeElasto::set_damping_coefficients(double axial, double edge, double flap, double torsion) {
    DampingCoefficients damping_coefficients;
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

double BladeElasto::get_mass() {
    double total_mass = 0.0;
    for (int ii = 0; ii < elements.size(); ii++) {
        total_mass += elements[ii]->GetMass();
    }
    return total_mass;
}

void BladeElasto::evaluate_position_rotation(ChVector<double>& position,
                                             ChQuaternion<double>& rotation,
                                             int element_index,
                                             double eta) {
    elements[element_index]->EvaluateSectionFrame(eta, position, rotation);
}

void BladeElasto::reset_loads() {
    for (int ii = 0; ii < nodes.size(); ii++) {
        nodes[ii]->SetForce(ChVector<double>(0.0, 0.0, 0.0));
        nodes[ii]->SetTorque(ChVector<double>(0.0, 0.0, 0.0));
    }
}

void BladeElasto::accumulate_element_load(ChVector<double> load, int element_index, double eta) {
    if (element_index >= elements.size() || element_index < 0) {
        throw std::runtime_error("Element index " + std::to_string(element_index) + " does not exist (max " +
                                 std::to_string(elements.size()) + ").");
    }
    auto element = elements[element_index];
    auto position = ChVector<double>(0.0, 0.0, 0.0);
    auto rotation = ChQuaternion<double>(0.0, 0.0, 0.0, 0.0);
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
