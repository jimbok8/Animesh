//
// Created by Dave Durbin on 18/5/20.
//

#include "PoSyOptimiser.h"
#include "PoSy.h"
#include <utility>
#include <vector>

using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, float>::GraphNode>;


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
void PoSyOptimiser::optimise_surfel(
        const std::shared_ptr<Surfel> &surfel_ptr,
        const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>& neighbour_data) const {
    using namespace std;
    using namespace Eigen;

    float weight = 0.0f;

    auto new_position = surfel_ptr->position;
    for (const auto& neighbour : neighbour_data) {
        float edge_weight = 1.0f;

        // TODO(dave.d) THIS IS ILLUSTRATIVE CODE ONLY AND NEEDS TO BE MADE CORRECT
        const auto & surfel_normal = surfel_ptr->frame_data.at(0).normal;
        const auto & surfel_tangent = surfel_ptr->tangent;

        new_position = average_posy_vectors(
                new_position,
                surfel_tangent,
                surfel_normal,
                edge_weight,
                get<0>(neighbour), // position
                get<1>(neighbour), // tangent
                get<2>(neighbour), // normal
                m_rho,
                weight
        );
        weight += edge_weight;
    }
    surfel_ptr->position = new_position;
}


void PoSyOptimiser::optimise_node(const SurfelGraphNodePtr& node) {
    using namespace std;
    using namespace Eigen;

    // Get neighbours
    auto neighbouring_nodes = m_surfel_graph.neighbours(node);
    vector<tuple<Vector3f, Vector3f, Vector3f>> neighbour_data{neighbouring_nodes.size()};
    for( const auto& neighbour : neighbouring_nodes ) {
        neighbour_data.emplace_back(
                neighbour->data()->position,
                neighbour->data()->tangent,
                Vector3f::UnitY());
    }

    // Optimise it
    optimise_surfel(node->data(), neighbour_data);
}