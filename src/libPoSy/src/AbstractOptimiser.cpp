//
// Created by Dave Durbin on 18/5/20.
//

#include "AbstractOptimiser.h"

#include <utility>
#include <sys/stat.h>
#include <algorithm>
#include <random>

using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, float>::GraphNode>;

AbstractOptimiser::AbstractOptimiser(Properties properties) : m_properties(std::move(properties)),
                                                              m_state{UNINITIALISED},
                                                              m_optimisation_cycles{0},
                                                              m_last_smoothness{0.0f},
                                                              m_convergence_threshold{1.0} {
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
        auto nodes_to_optimise = select_nodes_to_optimise();
        for (const auto &node : nodes_to_optimise) {
            optimise_node(node);
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

std::vector<SurfelGraphNodePtr>
AbstractOptimiser::select_nodes_to_optimise() const {
    using namespace std;

    assert(m_node_selection_function);
    return m_node_selection_function();
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


/*
 * total_neighbour_error = 0
 * for each neighbour
 *   total_neighbour_error += neighour_error
 * next
 * return total_neighbour_error / num neighours
 */
float
AbstractOptimiser::compute_surfel_error_for_frame(const std::shared_ptr<Surfel> &surfel,
                                              size_t frame_id) const {
    float total_neighbour_error = 0.0f;

    // Get all neighbours in frame
    const SurfelInFrame surfel_in_frame{surfel, frame_id};

    const auto &this_surfel_in_this_frame = m_norm_tan_by_surfel_frame.at(surfel_in_frame);

    const auto &bounds = m_neighbours_by_surfel_frame.equal_range(surfel_in_frame);
    unsigned int num_neighbours = 0;
    for (auto np = bounds.first; np != bounds.second; ++np) {
        const auto &this_neighbour_in_this_frame = m_norm_tan_by_surfel_frame.at(
                SurfelInFrame{np->second, frame_id});

        // Compute the error between this surfel in this frame and the neighbour in this frame.
        total_neighbour_error += compute_error(
                this_surfel_in_this_frame.normal, this_surfel_in_this_frame.tangent, surfel->closest_mesh_vertex_position,
                this_neighbour_in_this_frame.normal, this_neighbour_in_this_frame.tangent, np->second->closest_mesh_vertex_position);
        ++num_neighbours;
    }
    return (num_neighbours == 0)
           ? 0.0f
           : (total_neighbour_error / (float) num_neighbours);
}

/*
 * total_frame_error = 0
 * for each frame
 *   total_frame_error += compute_surfel_error_in_frame
 * next
 * total_surfel_error += (total_frame_error / num_frames)
 */
float
AbstractOptimiser::compute_surfel_error(const std::shared_ptr<Surfel> &surfel) const {
    float total_frame_error = 0.0f;

    // For each frame in which this surfel appears
    for (const auto &frame_data : surfel->frame_data) {
        // Compute the error in this frame
        total_frame_error += compute_surfel_error_for_frame(surfel, frame_data.pixel_in_frame.frame);
    }
    // Return mean error per frame
    (std::const_pointer_cast<Surfel>(surfel))->error = total_frame_error / surfel->frame_data.size();
    return surfel->error;
}


float
AbstractOptimiser::compute_mean_error_per_surfel() const {
    float total_error = 0.0f;
    for (const auto &n : m_surfel_graph.nodes()) {
        total_error += compute_surfel_error(n->data());
    }
    return total_error / m_surfel_graph.num_nodes();
}

/**
 * Check whether optimisation has converged.
 */
void
AbstractOptimiser::check_convergence() {
    using namespace spdlog;

    float current_smoothness = compute_mean_error_per_surfel();
    float improvement = m_last_smoothness - current_smoothness;
    m_last_smoothness = current_smoothness;
    float pct = (100.0f * improvement) / m_last_smoothness;
    info("Mean error per surfel: {:d} ({:f}%", current_smoothness, pct);
    if ((pct >= 0) && (std::abs(pct) < m_convergence_threshold)) {
        m_state = ENDING_OPTIMISATION;
    }
}

/**
 * Set the optimisation data
 */
void
AbstractOptimiser::set_data(const SurfelGraph &surfel_graph) {
    m_surfel_graph = surfel_graph;
    m_state = INITIALISED;
}

/**
 * Select all surfels in a layer and randomize the order
 */
std::vector<SurfelGraphNodePtr>
AbstractOptimiser::ssa_select_all_in_random_order() {
    using namespace std;

    vector<size_t> indices(m_surfel_graph.num_nodes());
    iota(begin(indices), end(indices), 0);
    shuffle(begin(indices), end(indices),
            default_random_engine(chrono::system_clock::now().time_since_epoch().count()));
    vector<SurfelGraphNodePtr> selected_nodes;
    selected_nodes.reserve(indices.size());
    const auto graph_nodes = m_surfel_graph.nodes();
    for (const auto i : indices) {
        selected_nodes.push_back(graph_nodes.at(i));
    }
    return selected_nodes;
}
