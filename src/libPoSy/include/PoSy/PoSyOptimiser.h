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
    void optimise_surfel(unsigned int surfel_idx) override;
    std::vector<size_t> select_surfels_to_optimise() override;

private:
    animesh::Graph<Surfel,int> m_surfel_graph;
};


#endif //ANIMESH_POSYOPTIMISER_H
