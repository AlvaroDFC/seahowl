#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

class Blade {
  public:
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> elements;
    std::vector<ChVector<double>> centers_reference;
    std::vector<ChVector2<double>> offsets_elastic;

    std::vector<ChVector2<double>> offsets_gravity;
    std::vector<double> structural_twist;
    std::vector<double> element_densities;
    std::vector<double> stiffness_axial;
    std::vector<double> stiffness_edge;
    std::vector<double> stiffness_flap;
    std::vector<double> stiffness_torsion;

    Blade();

    void make_blade(std::shared_ptr<ChMesh> mesh);
    void make_nodes(std::shared_ptr<ChMesh> mesh);
    void make_elements_tapered_timoshenko(std::shared_ptr<ChMesh> mesh);
};
