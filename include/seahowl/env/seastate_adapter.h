#pragma once

#include <iostream>
#include <cstring>
#include <memory>

#include <seahowl/env/wave_models.h>

namespace seahowl {

namespace env {
// forward declare (defined in .cpp file)
/**
 * @brief Interface to SeaState library.
 */
struct SeaStateLib;

/**
 * @brief Adapter to SeaState library.
 */
class SeaStateAdapter : public WaveModel {
  public:
    std::unique_ptr<SeaStateLib> pImpl;

    SeaStateAdapter(std::string SeaStateInfile);
    ~SeaStateAdapter();

    std::string get_seastate_infile() const { return seastate_infile; }

    void end();

    double get_water_level(const Vector3d& position, double time) const override;

  protected:
    double get_density_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_velocity_this(const Vector3d& position, double time) const override;
    virtual Vector3d get_acceleration_this(const Vector3d& position, double time) const override;
    std::string seastate_infile = "";
};

}  // namespace env
}  // namespace seahowl
