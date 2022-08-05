#include <seahowl/elasto/elasto.h>

#include <vector>
#include <numeric>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

void seahowl::elasto::ElastoFEAComponent::rotate(double angle, chrono::ChVector<double> axis) const {
    auto rotation = Q_from_AngAxis(angle, axis);
    for (auto& node : nodes) {
        auto new_position = rotation.Rotate(node->GetPos());
        node->SetPos(new_position);
        auto new_rotation = (rotation * node->GetRot()).GetNormalized();
        node->SetRot(new_rotation);
    }
}
void seahowl::elasto::ElastoFEAComponent::translate(chrono::ChVector<double> translation_vector) const {
    for (auto& node : nodes) {
        node->SetPos(node->GetPos() + translation_vector);
    }
}
double seahowl::elasto::ElastoFEAComponent::get_mass() const {
    return std::accumulate(
        cbegin(elements), cend(elements), 0.0,
        [](double total, decltype(elements)::value_type pElem) { return total += pElem->GetMass(); });
}
