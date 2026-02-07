#pragma once

// SEAHOWL headers
#include "seahowl/io/viz_insitu.h"

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}  // namespace irrlicht
}  // namespace chrono

namespace seahowl {
namespace io {
class VisualizationInSituIrrlicht : public VisualizationInSitu {
  public:
    VisualizationInSituIrrlicht();

    void initialize(seahowl::core::System& system) override;
    void initialize_elasto(seahowl::elasto::SystemElasto& system_elasto) override;
    void draw() override;

  private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application_irrlicht;
    std::shared_ptr<chrono::ChSystem> system_chrono;
};
}  // namespace io
}  // namespace seahowl
