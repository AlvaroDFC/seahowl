#pragma once

#include <seahowl/aero/reference_point_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/core/utils.h>

#include <chrono/core/ChVector.h>

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**@brief Blade aerodynamic element */
struct BladeElementAero {
    BladeReferencePointAero properties;        ///< Reference point (average properties of element)
    double length = 0.0;                       ///< Element length
    double pitch = 0.0;                        ///< Pitch angle
    double swept_annulus = 1.0;                ///< Annulus swept by element
    double chord_solidity = 1.0;               ///< Chord solidity (sigma)
    double induction_factor_axial = 0.0;       ///< Axial induction factor (first guess for next iteration)
    double induction_factor_tangential = 0.0;  ///< Tangential induction factor (first guess for next iteration)
    double radius = 0.0;                       ///< Radius (distance from hub)
    double distance_from_hub = 99999.9;        ///< Distance from hub (far by default for no effect)
    double distance_from_tip = 99999.9;        ///< Distance of element from tip (far by default for no effect)

    BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2);
    ~BladeElementAero();

    // ChVector2<double> get_induced_velocity_element(ChVector2<double>& local_velocity0);
    chrono::ChVector2<double> get_induced_velocity_rotor(chrono::ChVector2<double>& local_velocity_rotor0,
                                                         size_t nblades,
                                                         bool tip_loss = true,
                                                         bool hub_loss = true);
};

/**@brief Aerodynamic model for blade */
class BladeAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointAero> reference_points;    ///< Reference points as defined in input files
    std::vector<BladeReferencePointAero> discretized_points;  ///< Reference points interpolated at discretized points
    std::vector<BladeElementAero> elements;                   ///< Blade elements (between discretized points)
    std::vector<chrono::ChVector<double>> loads;              ///< Loads on blade elements
    std::vector<chrono::ChVector<double>>
        relative_velocities_induced;                        ///< Induced relative wind velocities on blade elements
    std::vector<chrono::ChVector<double>> wind_velocities;  ///< Uninduced wind velocities on blade elements
    std::vector<chrono::ChVector<double>>
        wind_velocities_shadowed;  ///< Wind velocities on blade elements with tower shadow effect
    double azimuth0 = 0.0;         ///< Initial azimuth of blade

    BladeAero();
    ~BladeAero();

    void build();
    void compute_distances_from_tip();
    void compute_distances_from_hub(chrono::ChVector<double> hub_apex_position, double hub_radius);
    void compute_radii(chrono::ChVector<double> hub_apex_position);
    chrono::ChVector<double> get_average_wind_velocity();
    chrono::ChVector<double> get_total_load();
    chrono::ChVector<double> get_total_load_barycenter();
    // void compute_wind_loads_bemt(WindModel& wind_model, double time);
};

}  // namespace aero
}  // namespace seahowl