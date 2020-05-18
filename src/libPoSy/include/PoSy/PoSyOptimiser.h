//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_POSYOPTIMISER_H
#define ANIMESH_POSYOPTIMISER_H

#include <Properties/Properties.h>
#include <Graph/Graph.h>
#include "AbstractOptimiser.h"
#include "../../../mesher/types.h"


class PoSyOptimiser : AbstractOptimiser {
public:
    /**
     * Construct a PoSyOptimiser.
     * @param properties Parameters for the optimiser.
     */
    explicit PoSyOptimiser(Properties properties);

    /**
     * Set the optimisation data
     */
     void set_data( animesh::Graph<Surfel,int>& surfel_graph );

    /**
     * Perform a single step of optimisation. Return true if converged or halted.
     */
    bool optimise_do_one_step();

private:
    animesh::Graph<Surfel,int> m_surfel_graph;
};


#endif //ANIMESH_POSYOPTIMISER_H
