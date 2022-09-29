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

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_positions() {
    std::vector<chrono::ChVector<double>> positions;
    for (auto& node : nodes) {
        positions.push_back(node->GetPos());
    }
    return positions;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_velocities() {
    std::vector<chrono::ChVector<double>> velocities;
    for (auto& node : nodes) {
        velocities.push_back(node->GetPos_dt());
    }
    return velocities;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_accelerations() {
    std::vector<chrono::ChVector<double>> accelerations;
    for (auto& node : nodes) {
        accelerations.push_back(node->GetPos_dtdt());
    }
    return accelerations;
}

std::vector<chrono::ChQuaternion<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_rotations() {
    std::vector<chrono::ChQuaternion<double>> rotations;
    for (auto& node : nodes) {
        rotations.push_back(node->GetRot());
    }
    return rotations;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_directions() {
    std::vector<chrono::ChVector<double>> directions;
    for (auto& node : nodes) {
        directions.push_back(node->GetRot().GetVector());
    }
    return directions;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_rotational_velocities() {
    std::vector<chrono::ChVector<double>> rotational_velocities;
    for (auto& node : nodes) {
        rotational_velocities.push_back(node->GetWvel_loc());
    }
    return rotational_velocities;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_rotational_accelerations() {
    std::vector<chrono::ChVector<double>> rotational_accelerations;
    for (auto& node : nodes) {
        rotational_accelerations.push_back(node->GetWacc_loc());
    }
    return rotational_accelerations;
}

std::vector<chrono::ChVector<double>> seahowl::elasto::ElastoFEAComponent::get_nodes_loads() {
    std::vector<chrono::ChVector<double>> loads;
    for (auto& node : nodes) {
        loads.push_back(node->GetForce());
    }
    return loads;
}