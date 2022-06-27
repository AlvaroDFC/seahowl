#include "blade.h"

Blade::Blade() {}

void Blade::make_blade(std::shared_ptr<ChMesh> mesh) {
    discretized_points = get_discretized_points(discretization_fractions, reference_points);
    make_nodes(mesh);
    make_elements_tapered_timoshenko(mesh);
};

void Blade::make_nodes(std::shared_ptr<ChMesh> mesh) {
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
        double twist = discretized_point.structural_twist * CH_C_PI / 180.0;  // convert degrees->radians
        ChMatrix33<> twist_matrix(Q_from_AngAxis(twist, node_axis));
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

void Blade::make_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh) {
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
    section->SetMassPerUnitLength(discretized_point.density);
    // axial
    section->SetAxialRigidity(discretized_point.stiffness_axial);
    section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
    // flap
    section->SetZbendingRigidity(discretized_point.stiffness_flap);
    // edge
    section->SetYbendingRigidity(discretized_point.stiffness_edge);
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
        section->SetMassPerUnitLength(discretized_point.density);
        // axial
        section->SetAxialRigidity(discretized_point.stiffness_axial);
        section->SetXtorsionRigidity(discretized_point.stiffness_torsion);
        // flap
        section->SetZbendingRigidity(discretized_point.stiffness_flap);
        // edge
        section->SetYbendingRigidity(discretized_point.stiffness_edge);
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

std::vector<BladeReferencePoint> get_discretized_points(std::vector<double>& discretization_fractions,
                                                        std::vector<BladeReferencePoint>& reference_points) {
    if (discretization_fractions.size() == 0) {
        // discretize at centers of reference directly
        return reference_points;
    } else {
        std::vector<BladeReferencePoint> discretized_points;
        std::vector<double> reference_fractions;
        for (int ii = 0; ii < reference_points.size(); ii++) {
            reference_fractions.push_back(reference_points[ii].fraction);
        }
        // check bounds
        if (reference_fractions[0] != 0.0 || reference_fractions[reference_fractions.size() - 1] != 1.0) {
            throw std::runtime_error("Reference fractions must start with 0 and end with 1 but got " +
                                     std::to_string(reference_fractions[0]) + " and " +
                                     std::to_string(reference_fractions[reference_fractions.size() - 1]) + ".");
        }
        for (int ii = 0; ii < discretization_fractions.size(); ii++) {
            // find position of node
            // interpolate to node position from centers of reference at key fractions
            double fraction = discretization_fractions[ii];
            // check bounds
            if (fraction < 0.0 || fraction > 1.0) {
                throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                         std::to_string(fraction) + ".");
            }
            int idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                      reference_fractions.begin();
            // decrease index for convenience
            idx -= 1;
            double fraction_lower = reference_fractions[idx];
            double fraction_upper = reference_fractions[idx + 1];
            double fraction_range = fraction_upper - fraction_lower;
            // check if fraction is same as lower or upper bound to avoid division by zero
            if (fraction_lower == fraction) {
                discretized_points.push_back(reference_points[idx]);
            } else if (fraction_upper == fraction) {
                discretized_points.push_back(reference_points[idx + 1]);
            } else {
                // get weighted point
                auto discretized_point =
                    (reference_points[idx] * (1.0 - (fraction - fraction_lower) / fraction_range) +
                     reference_points[idx + 1] * (1.0 - (fraction_upper - fraction) / fraction_range));
                discretized_points.push_back(discretized_point);
            }
        }
        return discretized_points;
    }
}
