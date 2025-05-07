#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/floater_elasto.h"  // TODO: create main body for floater hydro
#include "seahowl/fluid/hydro/floater_hydro.h"
// #include "seahowl/commons/component_fluid.h"

namespace seahowl {

namespace elasto {
class FloaterElasto;
}

namespace hydro {

// forward declare (defined in .cpp file)
/**
 * @brief Interface to HydroDyn library.
 */
struct HydroDynLib;

/**
 * @brief Adapter to HydroDyn library.
 */
class HydroDynAdapter {
  public:
    std::unique_ptr<seahowl::hydro::HydroDynLib> interface_hydrodyn;
    std::vector<Vector3d> forces_hydrodyn;
    std::vector<Vector3d> moments_hydrodyn;

    HydroDynAdapter();
    ~HydroDynAdapter();

    void set_infiles(const std::string& HydroDynInfile, const std::string& SeaStateInfile);
    void initialize(double time, double dt, const std::vector<EntityDynamic*>& nodes);
    void compute_loads(double time, const std::vector<EntityDynamic*>& nodes);
    void end();

  private:
    void update_nodes_motion(const std::vector<EntityDynamic*>& nodes);
};

/**
 * @brief Class for floater hydrodynamics with HydroDyn.
 *
 * Only works for floaters with a single body (or with ).
 */
class FloaterHydroDyn : public FloaterHydro {
  public:
    FloaterHydroDyn(const std::string& hydrodyn_filepath,
                    const std::string& seastate_filepath,
                    elasto::FloaterElasto& floater_elasto);

    void compute_env_loads(const env::EnvModel& env_model, double time);

    void initialize(double time, double dt) override;

  private:
    /** @brief HydroDyn adapter. */
    std::unique_ptr<seahowl::hydro::HydroDynAdapter> hydrodyn;

    /** @brief reference to elasto floater where information is extracted */
    elasto::FloaterElasto& floater_elasto;
};

}  // namespace hydro

}  // namespace seahowl
