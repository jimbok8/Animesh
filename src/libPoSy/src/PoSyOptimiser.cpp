//
// Created by Dave Durbin on 18/5/20.
//

#include "PoSyOptimiser.h"

#include <utility>

/**
 * Construct a PoSyOptimiser.
 * @param properties Parameters for the optimiser.
 */
PoSyOptimiser::PoSyOptimiser(Properties properties) : AbstractOptimiser(std::move(properties)) {

}

/**
 * Perform a single step of optimisation. Return true if converged or halted.
 */
bool PoSyOptimiser::optimise_do_one_step() {
    assert(m_state != UNINITIALISED);
    return false;
}
