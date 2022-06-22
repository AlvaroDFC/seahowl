#include "blade.h"

Blade::Blade() {}

void Blade::make_blade(std::shared_ptr<ChMesh> mesh) {
    make_nodes(mesh);
    make_elements_tapered_timoshenko(mesh);
};

void Blade::make_nodes(std::shared_ptr<ChMesh> mesh) {
    nodes.clear();
    int nnodes = centers_reference.size();
    for (int ii = 0; ii < nnodes; ii++) {
        auto node_pos = centers_reference[ii];

        // get node coordinate system
        ChVector<> node_axis;
        ChMatrix33<> node_rotation;
        if (ii == 0) {
            node_axis = (centers_reference[ii + 1] - node_pos).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - centers_reference[ii - 1]).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        } else {
            node_axis = (centers_reference[ii + 1] - centers_reference[ii - 1]).GetNormalized();
            node_rotation.Set_A_Xdir(node_axis, VECT_Y);
        }

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
    // offsets
    section->SetCenterOfMass(offsets_gravity[0].y(), -offsets_gravity[0].x());
    section->SetCentroidY(offsets_elastic[0].y());
    section->SetCentroidZ(-offsets_elastic[0].x());
    // material properties
    section->SetMassPerUnitLength(element_densities[0]);
    section->SetAxialRigidity(stiffness_axial[0]);
    // take structural twist into account trough trogonometry
    double twist = structural_twist[0] * CH_C_PI / 180.0;  // convert degrees->radians
    // flap
    section->SetYbendingRigidity(stiffness_edge[0] * abs(sin(twist)) + stiffness_flap[0] * abs(cos(twist)));
    // edge
    section->SetZbendingRigidity(stiffness_edge[0] * abs(cos(twist)) + stiffness_flap[0] * abs(sin(twist)));
    section->SetXtorsionRigidity(stiffness_torsion[0]);

    for (int ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = chrono_types::make_shared<ChElementBeamTaperedTimoshenko>();
        // add element to blade elements vector
        elements.push_back(element);
        // add element to mesh
        mesh->AddElement(element);
        // set element nodes
        element->SetNodes(nodes[ii - 1], nodes[ii]);
        // apply prebend
        element->SetNodeBreferenceRot((nodes[ii]->GetRot() * nodes[ii - 1]->GetRot().GetInverse()).GetNormalized());

        // create blade section
        auto blade_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        element->SetTaperedSection(blade_section);

        // set first section for tapered section
        blade_section->SetSectionA(section);

        // make second section for tapered section
        section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>();
        // offsets
        blade_section->SetSectionB(section);
        section->SetCenterOfMass(offsets_gravity[ii].y(), -offsets_gravity[ii].x());
        section->SetCentroidY(offsets_elastic[ii].y());
        section->SetCentroidZ(-offsets_elastic[ii].x());
        // material properties
        section->SetMassPerUnitLength(element_densities[ii]);
        section->SetAxialRigidity(stiffness_axial[ii]);
        // take structural twist into account trough trogonometry
        twist = structural_twist[ii] * CH_C_PI / 180.0;  // convert degrees->radians
        // flap
        section->SetYbendingRigidity(stiffness_edge[ii] * abs(sin(twist)) + stiffness_flap[ii] * abs(cos(twist)));
        // edge
        section->SetZbendingRigidity(stiffness_edge[ii] * abs(cos(twist)) + stiffness_flap[ii] * abs(sin(twist)));
        section->SetXtorsionRigidity(stiffness_torsion[ii]);
    }
}
