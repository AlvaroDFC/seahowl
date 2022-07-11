#include "blade.h"

#include "chrono/fea/ChBuilderBeam.h"

Blade::Blade() {}

void Blade::build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
    // blade
    discretized_points = get_discretized_points(discretization_elasto, reference_points);
    build_nodes(mesh);
    if (fpm_mode) {
        build_elements_tapered_timoshenko_fpm(mesh);
    } else {
        build_elements_tapered_timoshenko(mesh);
    }
    build_loads(system);
};

void Blade::build_nodes(std::shared_ptr<ChMesh> mesh) {
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

void Blade::build_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh) {
    elements.clear();
    int nelements = nodes.size() - 1;

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
void Blade::build_elements_tapered_timoshenko_fpm(std::shared_ptr<ChMesh> mesh) {
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

void Blade::build_loads(ChSystemSMC& system) {
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);
    for (int ii = 0; ii < elements.size(); ii++) {
        auto element = elements[ii];
        std::shared_ptr<ChLoad<ChLoaderWeighted>> loader_weighted(new ChLoad<ChLoaderWeighted>(element));
        loaders_aero.push_back(loader_weighted);
        loadcontainer->Add(loader_weighted);
    }
}

void Blade::rotate(double angle, ChVector<double> axis) {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}

void Blade::translate(ChVector<double> translation_vector) {
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        node->SetPos(node->GetPos() + translation_vector);
    }
}

void Blade::set_damping_coefficients(double axial, double edge, double flap, double torsion) {
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

double Blade::get_mass() {
    double total_mass = 0.0;
    for (int ii = 0; ii < elements.size(); ii++) {
        total_mass += elements[ii]->GetMass();
    }
    return total_mass;
}

std::vector<BladeAeroReferencePoint> Blade::get_aerodynamic_point_positions() {
    // build reference aero points at structural nodes
    std::vector<BladeAeroReferencePoint> nodal_positions;
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto node = nodes[ii];
        BladeAeroReferencePoint nodal_position;
        nodal_position.fraction = discretization_elasto[ii];
        nodal_position.coordinates = node->GetPos();
        // reference directions of cross-section
        auto local_direction_x = ChVector<double>(0.0, 1.0, 0.0);
        auto global_direction_x = node->TransformDirectionLocalToParent(local_direction_x);
        nodal_position.direction_x = global_direction_x;
        auto local_direction_y = ChVector<double>(0.0, 0.0, 1.0);
        auto global_direction_y = node->TransformDirectionLocalToParent(local_direction_y);
        nodal_position.direction_y = global_direction_y;
        // // tangential velocity
        // nodal_position.velocity = (node->GetPos_dt() ^ global_direction_x) * global_direction_x;
        nodal_position.velocity = node->GetPos_dt();
        nodal_positions.push_back(nodal_position);
    }
    // interpolate and return reference aero points at desired locations
    return get_discretized_points(discretization_aero, nodal_positions);
}
