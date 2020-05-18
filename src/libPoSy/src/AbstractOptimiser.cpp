//
// Created by Dave Durbin on 18/5/20.
//

#include "AbstractOptimiser.h"

#include <utility>
#include <sys/stat.h>

AbstractOptimiser::AbstractOptimiser(Properties properties) : m_properties(std::move(properties)),
                                                              m_state{UNINITIALISED},
                                                              m_optimisation_cycles{0} {
}

AbstractOptimiser::~AbstractOptimiser() = default;

/**
 * Start global smoothing.
 */
void
AbstractOptimiser::begin_optimisation() {
    assert(m_state == INITIALISED);

    m_optimisation_cycles = 0;
    optimisation_began();
    m_state = OPTIMISING;
}

/**
 * Perform post-smoothing tidy up.
 */
void
AbstractOptimiser::optimise_end() {
    assert(m_state == ENDING_OPTIMISATION);

    optimisation_ended();
    // TODO: Consider a final state here that can transition back to INITAILISED or make both READY
    m_state = INITIALISED;
}

/**
 * Perform a single step of optimisation. Return true if converged or halted.
 */
bool
AbstractOptimiser::optimise_do_one_step() {
    assert(m_state != UNINITIALISED);

    if (m_state == INITIALISED) {
        begin_optimisation();
    }

    if (m_state == OPTIMISING) {
        auto sto = select_surfels_to_optimise();
        for (auto surfel_idx : sto) {
            optimise_surfel(surfel_idx);
        }
        ++m_optimisation_cycles;
        check_cancellation();
        check_convergence();
    }

    if (m_state == ENDING_OPTIMISATION) {
        optimise_end();
        return true;
    }
    return false;
}


/**
 * Check whether the user cancelled optimisation by creating the
 * halt file.
 */
void
AbstractOptimiser::check_cancellation() {
    struct stat buffer{};
    auto rv = stat("halt", &buffer);
    if (rv == 0) {
        m_state = ENDING_OPTIMISATION;
    }
}

/**
 * Check whether optimisation has converged.
 */
void
AbstractOptimiser::check_convergence() {
    if (is_converged()) {
        m_state = ENDING_OPTIMISATION;
    }
}

/**
 * Set the optimisation data
 */
void
AbstractOptimiser::set_data(animesh::Graph<Surfel, int> &surfel_graph) {
    m_surfel_graph = surfel_graph;
    m_state = INITIALISED;
}
