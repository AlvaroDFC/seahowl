#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/elasto/floater_elasto.h"  // TODO: create main body for floater hydro
// #include "seahowl/hydro/morison.h"
// #include "seahowl/commons/component_fluid.h"

namespace seahowl {

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
    void initialize(double time, double dt, seahowl::elasto::FloaterElasto& floater);
    void compute_loads(double time, seahowl::elasto::FloaterElasto& floater);
    void end();

  private:
    void update_floater_body_motion(seahowl::elasto::FloaterElasto& floater);
};

}  // namespace hydro

}  // namespace seahowl
