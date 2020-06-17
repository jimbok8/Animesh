//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_ABSTRACTOPTIMISER_H
#define ANIMESH_ABSTRACTOPTIMISER_H

#include <Properties/Properties.h>
#include <Graph/Graph.h>
#include <Surfel/Surfel.h>

class AbstractOptimiser {
    using SurfelGraph = animesh::Graph<std::shared_ptr<Surfel>, float>;
    using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, float>::GraphNode>;

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
    void set_data(const SurfelGraph & graph);

protected:
    SurfelGraph m_surfel_graph;

private:
    Properties m_properties;
    unsigned int m_optimisation_cycles;

    std::function<std::vector<SurfelGraphNodePtr>(AbstractOptimiser&)> m_node_selection_function;

    std::vector<SurfelGraphNodePtr> select_nodes_to_optimise();

    void begin_optimisation();

    void optimise_end();

    void check_convergence();

    void check_cancellation();

    virtual bool is_converged() = 0;
    virtual void optimisation_began() = 0;
    virtual void optimisation_ended() = 0;
    virtual void optimise_node(const SurfelGraphNodePtr& node) = 0;
};

#endif //ANIMESH_ABSTRACTOPTIMISER_H
