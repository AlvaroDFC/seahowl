#pragma once

#include "../elasto/blade_elasto.h"
#include <seahowl/aero/blade_aero.h>
#include <seahowl/utils.h>

/**@brief wind turbine blade base class

Pattern <mediator> for elasto and aero 
*/
class Blade {
  public:
    std::shared_ptr<BladeElasto> elasto;
    std::shared_ptr<BladeAero> aero;
    std::vector<BladeReferencePoint> reference_points; 
    std::vector<DiscretizationPoint> mapping_aero2elasto;
    std::vector<DiscretizationPoint> mapping_elasto2aero;

    Blade();
    ~Blade() {}

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void set_discretization_elasto(std::vector<double> fractions);
    void set_discretization_aero(std::vector<double> fractions);
    void compute_mapping_aero2elasto();
    void compute_mapping_elasto2aero();
    void prestep(double time);
    void poststep(double time);

    /**@brief Compute aerodynamic loadings */
    void update_positions_aero();

    /**@brief Compute elastodynamic loadings */
    void update_loads_elasto();
};

