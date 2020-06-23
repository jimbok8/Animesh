//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_ABSTRACTOPTIMISER_H
#define ANIMESH_ABSTRACTOPTIMISER_H

#include <Properties/Properties.h>
#include <Graph/Graph.h>
#include <Surfel/Surfel.h>

#include <utility>

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
    void set_data(const SurfelGraph &graph);

protected:
    SurfelGraph m_surfel_graph;

    Properties m_properties;

    std::function<std::vector<SurfelGraphNodePtr>()> m_node_selection_function;

    /**
     * Select all surfels in a layer and randomize the order
     */
    std::vector<SurfelGraphNodePtr>
    ssa_select_all_in_random_order();

    float m_convergence_threshold;

private:
    // Utility class to map a surfel and frame
    struct SurfelInFrame {
        std::shared_ptr<Surfel> surfel_ptr;
        size_t frame_index;

        SurfelInFrame(std::shared_ptr<Surfel> surfel_ptr, size_t f)
                : surfel_ptr{std::move(surfel_ptr)},
                  frame_index{f} {
        }

        // Sort By surfel ID
        bool operator<(const SurfelInFrame &other) const {
            if (frame_index != other.frame_index)
                return frame_index < other.frame_index;

            return surfel_ptr->id < other.surfel_ptr->id;
        }
    };

    struct NormalTangent {
        Eigen::Vector3f normal;
        Eigen::Vector3f tangent;

        NormalTangent(Eigen::Vector3f n, Eigen::Vector3f t) : normal{std::move(n)}, tangent{std::move(t)} {
        }
    };


    unsigned int m_optimisation_cycles;

    // Error and convergence
    float m_last_smoothness;

    float compute_mean_error_per_surfel() const;

    float compute_surfel_error(const std::shared_ptr<Surfel> &surfel) const;

    float compute_surfel_error_for_frame(const std::shared_ptr<Surfel> &surfel, size_t frame_id) const;

    void check_convergence();

    /**
     * Useful cache for error computation. Stores a list of surfels which are neighbours of the given surfels in frame
     * Key is surfel, frame
     */
    std::multimap<SurfelInFrame, std::shared_ptr<Surfel>> m_neighbours_by_surfel_frame;

    /**
     * Useful cache for error computation. Recalculated per level.
     * A map from a surfel/frame pair to that surfel's transformed normal, tangent and position in that frame.
     */
    std::map<SurfelInFrame, NormalTangent> m_norm_tan_by_surfel_frame;

    std::vector<SurfelGraphNodePtr> select_nodes_to_optimise() const;

    void begin_optimisation();

    void optimise_end();


    void check_cancellation();

    virtual void optimisation_began() = 0;

    virtual void optimisation_ended() = 0;

    virtual void optimise_node(const SurfelGraphNodePtr &node) = 0;

    virtual float
    compute_error(const Eigen::Vector3f &normal1, const Eigen::Vector3f &tangent1, const Eigen::Vector3f &position1,
                  const Eigen::Vector3f &normal2, const Eigen::Vector3f &tangent2,
                  const Eigen::Vector3f &position2) const = 0;
};

#endif //ANIMESH_ABSTRACTOPTIMISER_H
