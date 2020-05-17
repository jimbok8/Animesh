//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_POSYOPTIMISER_H
#define ANIMESH_POSYOPTIMISER_H

#include <Properties/Properties.h>
#include "AbstractOptimiser.h"


class PoSyOptimiser : AbstractOptimiser {
public:
    /**
     * Construct a PoSyOptimiser.
     * @param properties Parameters for the optimiser.
     */
    explicit PoSyOptimiser(Properties properties);

    /**
     * Perform a single step of optimisation. Return true if converged or halted.
     */
    bool optimise_do_one_step();

private:
};


#endif //ANIMESH_POSYOPTIMISER_H
