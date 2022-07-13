#include "utils.h"

std::vector<DiscretizationPoint> get_indice_and_positions(std::vector<double>& discretization_fractions,
                                                          std::vector<double>& reference_fractions) {
    std::vector<DiscretizationPoint> points;
    for (int ii = 0; ii < discretization_fractions.size(); ii++) {
        double fraction = discretization_fractions[ii];
        // check bounds
        if (fraction < 0.0 || fraction > 1.0) {
            throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                     std::to_string(fraction) + ".");
        }
        int idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                  reference_fractions.begin();
        // decrease index for getting lower bound
        idx -= 1;
        double fraction_lower = reference_fractions[idx];
        double fraction_upper = reference_fractions[idx + 1];
        double fraction_range = fraction_upper - fraction_lower;
        double eta = 2.0 * (fraction - fraction_lower) / fraction_range - 1.0;
        DiscretizationPoint point;
        point.index = idx;
        point.eta = eta;
        points.push_back(point);
    }
    return points;
}
