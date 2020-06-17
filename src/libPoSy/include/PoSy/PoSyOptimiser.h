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
    bool is_converged() override;
    void optimisation_began() override;
    void optimisation_ended() override;
    void optimise_node(const SurfelGraphNodePtr& node) override;

private:
    float m_rho;

    void optimise_surfel(
            const std::shared_ptr<Surfel> &surfel_ptr,
            const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>& neighbour_data) const;
};


#endif //ANIMESH_POSYOPTIMISER_H
