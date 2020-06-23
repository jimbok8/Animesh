//
// Created by Dave Durbin on 18/5/20.
//

#include "PoSyOptimiser.h"
#include "PoSy.h"
#include <utility>
#include <vector>
#include <RoSy/RoSy.h>

using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, float>::GraphNode>;

static const char *SSA_SELECT_ALL_IN_RANDOM_ORDER = "select-all-in-random-order";
static const char *SSA_SELECT_WORST_100 = "select-worst-100";

// ========
/**
 * Construct a PoSyOptimiser.
 * @param properties Parameters for the optimiser.
 */
PoSyOptimiser::PoSyOptimiser(Properties properties) : AbstractOptimiser(std::move(properties)) {
    m_rho = m_properties.getFloatProperty("rho");
    m_convergence_threshold = m_properties.getFloatProperty("convergence-threshold");
    auto ssa = m_properties.getProperty("surfel-selection-algorithm");
    if (ssa == SSA_SELECT_ALL_IN_RANDOM_ORDER) {
        m_node_selection_function = bind(&PoSyOptimiser::ssa_select_all_in_random_order, this);
    } else if (ssa == SSA_SELECT_WORST_100) {
        m_node_selection_function = bind(&PoSyOptimiser::ssa_select_worst_100, this);
    } else {
        throw std::runtime_error("Unknown surfel selection algorithm " + ssa);
    }
}

PoSyOptimiser::~PoSyOptimiser() = default;


void PoSyOptimiser::optimisation_began() {

}

void PoSyOptimiser::optimisation_ended() {

}

/**
 * Apply following smoothing operation to this Surfel

 for each neighbour N of v
    compute 9 plausible positions to compare with neighbours
    Pv = {pv , pv + ov, pv + ov', pv + ov + ov', pv - ov + ov' etc.}

    compute 9 plausible positions of neighbour
    PN = { pN , pN + oN , pN + oN', pN + oN + oN', pN - oN + oN' etc.}

    brute force the closest pair of points {vc, Nc} in Pv and PN
    compute d = Nc - vc
    add weighted proportion of d to pv
  next neighbour

 */
void PoSyOptimiser::optimise_surfel(
        const std::shared_ptr<Surfel> &surfel_ptr,
        const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>> &neighbour_data) const {
    using namespace std;
    using namespace Eigen;

    float weight = 0.0f;

    auto new_position = surfel_ptr->closest_mesh_vertex_position;
    for (const auto &neighbour : neighbour_data) {
        float edge_weight = 1.0f;

        const auto &surfel_normal = surfel_ptr->frame_data.at(0).normal;
        const auto &surfel_tangent = surfel_ptr->tangent;

        new_position = average_posy_vectors(
                new_position,
                surfel_tangent,
                surfel_normal,
                edge_weight,
                get<0>(neighbour), // position
                get<1>(neighbour), // tangent
                get<2>(neighbour), // normal
                m_rho,
                weight
        );
        weight += edge_weight;
    }
    surfel_ptr->closest_mesh_vertex_position = new_position;
}

/**
 * Surfel selection model 2: Select top 100 error scores
 */
std::vector<SurfelGraphNodePtr>
PoSyOptimiser::ssa_select_worst_100() {
    using namespace std;

    vector<SurfelGraphNodePtr> selected_nodes;
    for (const auto &n : m_surfel_graph.nodes()) {
        selected_nodes.push_back(n);
    }
    stable_sort(begin(selected_nodes), end(selected_nodes),
                [this](const SurfelGraphNodePtr &s1, const SurfelGraphNodePtr &s2) {
                    return s1->data()->position_error > s2->data()->position_error;
                });

    selected_nodes.resize(100);
    return selected_nodes;
}

/**
 * Return a vector of pairs of FrameData for each frame that these surfels have in common
 */
std::vector<std::pair<std::reference_wrapper<const FrameData>, std::reference_wrapper<const FrameData>>>
find_common_frames_for_surfels(const std::shared_ptr<Surfel> &surfel1, const std::shared_ptr<Surfel> &surfel2) {
    using namespace std;

    vector<reference_wrapper<const FrameData>> surfel1_frames;
    vector<reference_wrapper<const FrameData>> surfel2_frames;
    for (const auto &f : surfel1->frame_data) {
        surfel1_frames.emplace_back(f);
    }
    for (const auto &f : surfel2->frame_data) {
        surfel2_frames.emplace_back(f);
    }
    sort(surfel1_frames.begin(), surfel1_frames.end(),
         [](const FrameData &f1, const FrameData &f2) { return f1.pixel_in_frame.frame < f2.pixel_in_frame.frame; });
    sort(surfel2_frames.begin(), surfel2_frames.end(),
         [](const FrameData &f1, const FrameData &f2) { return f1.pixel_in_frame.frame < f2.pixel_in_frame.frame; });

    vector<pair<reference_wrapper<const FrameData>, reference_wrapper<const FrameData>>> common_frames;
    auto it1 = surfel1->frame_data.begin();
    auto it2 = surfel2->frame_data.begin();
    while (it1 != surfel1->frame_data.end() && it2 != surfel2->frame_data.end()) {
        if (it1->pixel_in_frame.frame < it2->pixel_in_frame.frame) {
            ++it1;
        } else if (it2->pixel_in_frame.frame < it1->pixel_in_frame.frame) {
            ++it2;
        } else {
            common_frames.emplace_back(ref(*it1), ref(*it2));
            ++it1;
            ++it2;
        }
    }
    return common_frames;
}


/**
 * Populate positions, tangents and normals for all eligible surfel neighbours
 * pos/tan/norm is eligible iff the neighbour and surfel share a common frame
 * pos/tan/norm are converted to the orignating surfel's frame of reference.
 */
std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>>
PoSyOptimiser::get_neighbouring_data(const SurfelGraphNodePtr &node) {
    using namespace std;
    using namespace Eigen;

    vector<tuple<Vector3f, Vector3f, Vector3f>> eligible_neighbour_pos_tan_norms;

    // For each neighbour
    size_t total_common_frames = 0;
    const auto &this_surfel_ptr = node->data();
    for (const auto &surfel_ptr_neighbour : m_surfel_graph.neighbours(node)) {
        const auto that_surfel_ptr = surfel_ptr_neighbour->data();
        const auto common_frames = find_common_frames_for_surfels(this_surfel_ptr, that_surfel_ptr);
        total_common_frames += common_frames.size();

        // For each common frame, get position, normal and tangent in surfel space
        for (auto const &frame_pair : common_frames) {
            auto surfel_to_frame = frame_pair.first.get().transform;
            auto frame_to_surfel = surfel_to_frame.transpose();
            auto neighbour_to_frame = frame_pair.second.get().transform;
            auto neighbour_to_surfel = frame_to_surfel * neighbour_to_frame;

            // Get neighbour tangent, pos and normal in surfel space
            // So we need:
            //    transform from free space to frame space for tangent (stored) (we already have normal in frame space)
            //	  transform from frame space to free space using surfel data. (inv of stored)
            auto neighbour_tan_in_surfel_space = neighbour_to_surfel * that_surfel_ptr->tangent;
            auto neighbour_normal_in_frame = frame_pair.second.get().normal;
            auto neighbour_norm_in_surfel_space = frame_to_surfel * neighbour_normal_in_frame;

            // Surfel 'position' field is the absolute position of the nearest mesh vertex in Surfel space
            // i.e assuming v = (0,0,0)
            auto neighbour_pos_in_frame = (neighbour_to_frame * that_surfel_ptr->closest_mesh_vertex_position) +
                                          frame_pair.second.get().position;
            auto neighbour_pos_in_surfel_space =
                    frame_to_surfel * (neighbour_pos_in_frame - frame_pair.first.get().position);

            eligible_neighbour_pos_tan_norms.emplace_back(
                    neighbour_pos_in_surfel_space,
                    neighbour_norm_in_surfel_space,
                    neighbour_tan_in_surfel_space);
        }
    }
    return eligible_neighbour_pos_tan_norms;
}

void PoSyOptimiser::optimise_node(const SurfelGraphNodePtr &node) {
    auto neighbouring_data = get_neighbouring_data(node);
    optimise_surfel(node->data(), neighbouring_data);
}

float
PoSyOptimiser::compute_error(
        const Eigen::Vector3f &normal1, const Eigen::Vector3f &tangent1, const Eigen::Vector3f &position1,
        const Eigen::Vector3f &normal2, const Eigen::Vector3f &tangent2, const Eigen::Vector3f &position2) const {
    using namespace std;
    using namespace Eigen;

    // parameter order in RoSy is tangent, normal
    auto best_pair = best_rosy_vector_pair(tangent1, normal1, tangent2, normal2);
    const auto l1 = compute_local_lattice_vertices(position1, normal1, best_pair.first, m_rho);
    const auto l2 = compute_local_lattice_vertices(position2, normal2, best_pair.second, m_rho);
    const auto tuple = closest_points(l1, l2);
    return std::get<4>(tuple);
}
