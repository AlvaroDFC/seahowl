#ifndef BLADE_CORE_H_
#define BLADE_CORE_H_

#include "../elasto/blade_elasto.h"
#include "../aero/blade_aero.h"

class Blade {
  public:
    std::shared_ptr<BladeElasto> elasto;
    std::shared_ptr<BladeAero> aero;
    std::vector<BladeReferencePoint> reference_points;

    Blade() {
        elasto = std::make_shared<BladeElasto>();
        aero = std::make_shared<BladeAero>();
    }

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh) {
        // push reference points
        elasto->reference_points.clear();
        /* aero->reference_points.clear(); */
        for (int ii = 0; ii < reference_points.size(); ii++) {
            elasto->reference_points.push_back(BladeReferencePointElasto(reference_points[ii]));
            /* aero->reference_points.push_back(BladeReferencePointAero(reference_points[ii])); */
        }
        // build elasto
        elasto->build(system, mesh);
    }

    void set_discretization_elasto(std::vector<double> fractions) { elasto->discretization_fractions = fractions; };

    void set_discretization_aero(std::vector<double> fractions) { aero->discretization_fractions = fractions; };

    /* std::vector<BladeAeroReferencePoint> BladeElasto::get_aerodynamic_point_positions() { */
    /*     // build reference aero points at structural nodes */
    /*     std::vector<BladeAeroReferencePoint> nodal_positions; */
    /*     for (int ii = 0; ii < nodes.size(); ii++) { */
    /*         auto node = nodes[ii]; */
    /*         BladeAeroReferencePoint nodal_position; */
    /*         nodal_position.fraction = discretization_elasto[ii]; */
    /*         nodal_position.coordinates = node->GetPos(); */
    /*         // reference directions of cross-section */
    /*         auto local_direction_x = ChVector<double>(0.0, 1.0, 0.0); */
    /*         auto global_direction_x = node->TransformDirectionLocalToParent(local_direction_x); */
    /*         nodal_position.direction_x = global_direction_x; */
    /*         auto local_direction_y = ChVector<double>(0.0, 0.0, 1.0); */
    /*         auto global_direction_y = node->TransformDirectionLocalToParent(local_direction_y); */
    /*         nodal_position.direction_y = global_direction_y; */
    /*         // // tangential velocity */
    /*         // nodal_position.velocity = (node->GetPos_dt() ^ global_direction_x) * global_direction_x; */
    /*         nodal_position.velocity = node->GetPos_dt(); */
    /*         nodal_positions.push_back(nodal_position); */
    /*     } */
    /*     // interpolate and return reference aero points at desired locations */
    /*     return get_discretized_points(discretization_aero, nodal_positions); */
    /* } */
};

#endif  // BLADE_CORE_H_
