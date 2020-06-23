//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_POSYOPTIMISER_H
#define ANIMESH_POSYOPTIMISER_H

#include <Properties/Properties.h>
#include <Graph/Graph.h>
#include "AbstractOptimiser.h"
#include "../../../mesher/types.h"


class PoSyOptimiser : public AbstractOptimiser {
    using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, float>::GraphNode>;

public:
    /**
     * Construct a PoSyOptimiser.
     * @param properties Parameters for the optimiser.
     */
    explicit PoSyOptimiser(Properties properties);

    ~PoSyOptimiser() override;

protected:
    void optimisation_began() override;

    void optimisation_ended() override;

    void optimise_node(const SurfelGraphNodePtr &node) override;

    /**
     * Surfel selection model 2: Select top 100 error scores
     */
    std::vector<SurfelGraphNodePtr>
    ssa_select_worst_100();

private:
    float m_rho;


    /**
     * Populate positions, tangents and normals for all eligible surfel neighbours
     * pos/tan/norm is eligible iff the neighbour and surfel share a common frame
     * pos/tan/norm are converted to the orignating surfel's frame of reference.
     */
    std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>
    get_neighbouring_data(const SurfelGraphNodePtr &node);

    void optimise_surfel(
            const std::shared_ptr<Surfel> &surfel_ptr,
            const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>> &neighbour_data) const;

    float
    compute_error(const Eigen::Vector3f &normal1, const Eigen::Vector3f &tangent1, const Eigen::Vector3f &position1,
                  const Eigen::Vector3f &normal2, const Eigen::Vector3f &tangent2,
                  const Eigen::Vector3f &position2) const override;
};


#endif //ANIMESH_POSYOPTIMISER_H
