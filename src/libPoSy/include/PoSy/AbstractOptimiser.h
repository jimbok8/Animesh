//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_ABSTRACTOPTIMISER_H
#define ANIMESH_ABSTRACTOPTIMISER_H

#include <Properties/Properties.h>
#include <Graph/Graph.h>
#include <Surfel/Surfel.h>

class AbstractOptimiser {
public:
    explicit AbstractOptimiser(Properties properties);

    virtual ~AbstractOptimiser();

    /**
     * State of the optimiser.
     */
    enum OptimisationState {
        UNINITIALISED,
        INITIALISED,
        OPTIMISING,
        ENDING_OPTIMISATION
    } m_state;

    /**
     * Perform a single step of optimisation. Return true if converged or halted.
     */
    bool optimise_do_one_step();

    /**
     * Set the optimisation data
     */
    void set_data(animesh::Graph<std::shared_ptr<Surfel>, int> &surfel_graph);

protected:
    animesh::Graph<std::shared_ptr<Surfel>, int> m_surfel_graph;

private:
    Properties m_properties;
    unsigned int m_optimisation_cycles;

    void begin_optimisation();

    void optimise_end();

    void check_convergence();

    void check_cancellation();

    virtual bool is_converged() = 0;
    virtual void optimisation_began() = 0;
    virtual void optimisation_ended() = 0;
    virtual void optimise_surfel(std::shared_ptr<Surfel> surfel_ptr) = 0;
    virtual std::vector<std::shared_ptr<Surfel>> select_surfels_to_optimise() = 0;
};

#endif //ANIMESH_ABSTRACTOPTIMISER_H
