#pragma once

#include "seahowl/fluid/turbine_fluid.h"
#include "seahowl/commons/component_fluid.h"

#include <vector>
#include <deque>

namespace seahowl {
namespace fluid {

/**
 * @brief Fluid system base class.
 */
class SystemFluid : public ComponentFluid {
  public:
    /** @brief Turbines in system. */
    std::deque<std::shared_ptr<TurbineFluid>> turbines{};
    /** @brief Components in system. */
    std::deque<std::shared_ptr<ComponentFluid>> components{};

    /**
     * @brief Builds the system.
     */
    void build() override;

    /**
     * @brief Computes fluid loads on system.
     *
     * @param[in] env_model env model to use for applying fluid loads.
     * @param[in] time Time of simulation.
     */
    void compute_env_loads(const env::EnvModel& env_model, double time) override;

    /**
     * @brief Adds turbine to system.
     *
     * @param[in] turbine Turbine to add to system.
     */
    void add(std::shared_ptr<TurbineFluid> turbine);

    /**
     * @brief Adds component to system.
     *
     * @param[in] component Component to add to system.
     */
    void add(std::shared_ptr<seahowl::ComponentFluid> component);
};

// Backward-compatible alias
using SystemAero = SystemFluid;

}  // namespace fluid
}  // namespace seahowl
