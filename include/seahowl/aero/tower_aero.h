#pragma once

#include <vector>

#include <seahowl/aero/reference_point_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/core/utils.h>

#include <chrono/core/ChVector.h>

namespace seahowl {

/**@brief Aerodynamic module */
namespace aero {

/**@brief Tower aerodynamic element */
struct TowerElementAero {
    TowerReferencePointAero properties;  ///< Reference point (average properties of element)
    double length = 0.0;                 ///< Element length

    TowerElementAero(TowerReferencePointAero& point1, TowerReferencePointAero& point2);
    ~TowerElementAero();
};

/**@brief Aerodynamic model for blade */
class TowerAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<TowerReferencePointAero> reference_points;    ///< Reference points as defined in input files
    std::vector<TowerReferencePointAero> discretized_points;  ///< Reference points interpolated at discretized points
    std::vector<TowerElementAero> elements;                   ///< Tower elements (between discretized points)
    std::vector<chrono::ChVector<double>> loads;              ///< Loads on blade elements
    void compute_wind_loads_morison(WindModel& wind_model, double time);

    TowerAero();
    ~TowerAero();

    void build();
};

}  // namespace aero
}  // namespace seahowl
