//
// Created by Dave Durbin on 18/5/20.
//

#include "PoSyOptimiser.h"
#include "PoSy.h"
#include <utility>
#include <vector>


// ========
/**
 * Construct a PoSyOptimiser.
 * @param properties Parameters for the optimiser.
 */
PoSyOptimiser::PoSyOptimiser(Properties properties) : AbstractOptimiser(std::move(properties)) {
    m_rho = properties.getFloatProperty("rho");
}

PoSyOptimiser::~PoSyOptimiser() = default;


bool PoSyOptimiser::is_converged() {
    return true;
}

void PoSyOptimiser::optimisation_began() {

}

void PoSyOptimiser::optimisation_ended() {

}

/**
 * Apply following smoothing operation to this Surfel

 for each neighbour N of v
    compute 9 plausible positions to compare with neighbours
    Pv = {pv , pv + ov, pv + ov', pv + ov + ov', pv - ov + ov' etc.}

    compute 9 plausible positions of neighbour
    PN = { pN , pN + oN , pN + oN', pN + oN + oN', pN - oN + oN' etc.}

    brute force the closest pair of points {vc, Nc} in Pv and PN
    compute d = Nc - vc
    add weighted proportion of d to pv
  next neighbour

 */
void PoSyOptimiser::optimise_surfel(const std::shared_ptr<Surfel> &surfel_ptr) {
    using namespace std;

    float weight = 0.0f;

    auto new_position = surfel_ptr->position;
    for (const auto neighbour_node : m_surfel_graph.neighbours(surfel_ptr)) {
        float edge_weight = 1.0f;

        const Eigen::Vector3f surfel_normal;
        const Eigen::Vector3f surfel_tangent;
        const Eigen::Vector3f neighbour_normal;
        const Eigen::Vector3f neighbour_tangent;

        new_position = average_posy_vectors(
                new_position,
                surfel_tangent,
                surfel_normal,
                edge_weight,
                neighbour_node->data()->position,
                neighbour_tangent,
                neighbour_normal,
                m_rho,
                weight
        );
        weight += edge_weight;
    }
    surfel_ptr->position = new_position;
}

std::vector<std::shared_ptr<Surfel>> PoSyOptimiser::select_surfels_to_optimise() {
    return std::vector<std::shared_ptr<Surfel>>{};
}

