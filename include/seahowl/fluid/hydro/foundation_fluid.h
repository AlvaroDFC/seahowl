#pragma once

#include "seahowl/fluid/component_fluid.h"

namespace seahowl {
namespace fluid {

/**@brief Hydrodynamic module */
namespace hydro {

class FoundationFluid : public virtual ComponentFluid {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~FoundationFluid() = default;
};

}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl
