#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <Properties/Properties.h>
#include <Surfel/Surfel.h>
#include <Surfel/PixelInFrame.h>
#include <DepthMap/DepthMap.h>
#include <omp.h>
#include "correspondences_io.h"
#include "types.h"
#include "utilities.h"
#include "depth_map_io.h"

const char dm_template[] = "depth_map_genned_L%02d_F%02d.pgm";
const char norm_template[] = "normal_map_genned_L%02d_F%02d.ppm";

/**
 * Create a map to allow lookup of a Surfel ID from one of its PIF
 */
std::map<PixelInFrame, std::shared_ptr<Surfel>>
map_pifs_to_surfel(const std::vector<std::shared_ptr<Surfel>> &surfels) {
    using namespace std;

    map<PixelInFrame, std::shared_ptr<Surfel>> pif_to_surfel;
    for (const auto &surfel_ptr : surfels) {
        for (const auto &frame_data : surfel_ptr->frame_data) {
            pif_to_surfel.emplace(frame_data.pixel_in_frame, surfel_ptr);
        }
    }
    return pif_to_surfel;
}

namespace std {
    template<>
    struct std::less<std::reference_wrapper<Surfel>> {
        bool operator()(const reference_wrapper<Surfel> &k1, const reference_wrapper<Surfel> &k2) const {
            return k1.get().id < k2.get().id;
        }
    };
}

/**
 * Given a set of parent surfels and child surfels, estalish a mapping from child to one or more parents.
 * Children with no parents are stored as IDs in unmapped
 */
std::multimap<std::shared_ptr<Surfel>, std::shared_ptr<Surfel>>
compute_child_to_parent_surfel_map(const std::vector<std::shared_ptr<Surfel>> &child_surfels, //
                                   const std::vector<std::shared_ptr<Surfel>> &parent_surfels, //
                                   std::vector<std::shared_ptr<Surfel>> &orphans) {
    using namespace std;

    // Construct a map from parent level PIF to Surfel id
    map<PixelInFrame, shared_ptr<Surfel>> pif_to_parent = map_pifs_to_surfel(parent_surfels);

    // For each PIF in each child surfel, try to find a matching PIF in the parent surfels map
    multimap<shared_ptr<Surfel>, shared_ptr<Surfel>> child_to_parents_surfel_map;

    for (const auto &child_surfel_ptr : child_surfels) {
        unsigned int parents_found = 0;
        for (const auto &child_frame : child_surfel_ptr->frame_data) {
            PixelInFrame parent_pif{child_frame.pixel_in_frame.pixel.x / 2, //
                                    child_frame.pixel_in_frame.pixel.y / 2, //
                                    child_frame.pixel_in_frame.frame};
            const auto &it = pif_to_parent.find(parent_pif);
            if (it != pif_to_parent.end()) {
                child_to_parents_surfel_map.emplace(child_surfel_ptr, it->second);
                ++parents_found;
            } // else parent not found for this PIF
        }
        if (parents_found == 0) {
            orphans.push_back(child_surfel_ptr);
        }
    }
    return child_to_parents_surfel_map;
}

/**
 * Initialise the child surfel tangents from their psarwent surfel tangents
 * where the parent-child mappings are defined in child_to_parents
 */
void
down_propagate_tangents(const std::multimap<std::shared_ptr<Surfel>, std::shared_ptr<Surfel>> &child_to_parents) {

    using namespace std;
    using namespace Eigen;
    cout << "Propagating surfel tangents from previous level" << endl;

    // for each surfel in the lower level
    auto child_iterator = child_to_parents.begin();
    while (child_iterator != child_to_parents.end()) {
        auto child_ptr = child_iterator->first;
        int num_parents = 0;
        Vector3f mean_tangent{0.0f, 0.0f, 0.0};
        auto parent_iterator = child_iterator;
        while ((parent_iterator != child_to_parents.end()) && (parent_iterator->first == child_ptr)) {
            mean_tangent += parent_iterator->second->tangent;
            ++num_parents;
            ++parent_iterator;
        }
        child_iterator->first->tangent = (mean_tangent / num_parents).normalized();
        child_iterator = parent_iterator;
    }
}

std::vector<std::vector<PixelInFrame>>
get_correspondences(const Properties &properties,
                    unsigned int level,
                    const std::vector<DepthMap> &depth_map,
                    std::vector<Camera> &cameras) {
    using namespace std;

    vector<vector<PixelInFrame>> correspondences;

    if (properties.getBooleanProperty("load-correspondences")) {
        string corr_file_template = properties.getProperty("correspondence-file-template");
        size_t size = snprintf(nullptr, 0, corr_file_template.c_str(), level) + 1; // Extra space for '\0'
        std::unique_ptr<char[]> buf(new char[size]);
        snprintf(buf.get(), size, corr_file_template.c_str(), level);
        string file_name = std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside

        load_correspondences_from_file(file_name, correspondences);
    } else {
        cout << "Computing correspondences from scratch" << endl;
        throw runtime_error("Not implemented. Copy code from dm_to_pointcloud");
    }

    if (correspondences.empty()) {
        throw runtime_error("No correspondences found");
    }
    return correspondences;
}


void
maybe_save_depth_and_normal_maps(const Properties &properties,
                                 const std::vector<std::vector<DepthMap>> &depth_map_hierarchy) {
    using namespace std;

    int num_levels = depth_map_hierarchy.size();
    int num_frames = depth_map_hierarchy.at(0).size();
    if (properties.getBooleanProperty("dump-depth-maps")) {
        cout << " Dumping depth maps" << endl;
        for (unsigned int level = 0; level < num_levels; ++level) {
            for (unsigned int frame = 0; frame < num_frames; ++frame) {
                auto dm_file_name = file_name_from_template_level_and_frame(dm_template, level, frame);
                save_depth_map_as_pgm(dm_file_name, depth_map_hierarchy.at(level).at(frame));
                auto norm_file_name = file_name_from_template_level_and_frame(norm_template, level, frame);
                save_normals_as_ppm(norm_file_name, depth_map_hierarchy.at(level).at(frame));
            }
        }
    }
}

/**
 * Remove the previous level surfels from the Surfel::map
 */
void
unmap_surfels(const std::vector<std::shared_ptr<Surfel>> &surfels) {
    using namespace std;

    for (const auto &surfel_ptr : surfels) {
        Surfel::surfel_by_id.erase(surfel_ptr->id);
    }
}

/**
 * Given a list of surfel IDs, remove them from the surfel list.
 * The properties object is consulted to check whether to log this removal or not.
 */
void
remove_surfels_by_id(std::vector<std::shared_ptr<Surfel>> &surfels,
                     std::vector<std::shared_ptr<Surfel>> &surfels_to_remove,
                     const Properties &properties) {
    using namespace std;

    size_t initial_surfel_count = surfels.size();
    if (!surfels_to_remove.empty()) {
        sort(surfels_to_remove.begin(),
             surfels_to_remove.end(),
             [](std::shared_ptr<Surfel> &s1, std::shared_ptr<Surfel> &s2) {
                 return s1->id < s2->id;
             });
        surfels.erase(remove_if(
                begin(surfels), end(surfels),
                [&](const std::shared_ptr<Surfel> &s) {
                    return binary_search(
                            begin(surfels_to_remove), end(surfels_to_remove),
                            s,
                            [](const std::shared_ptr<Surfel> &s1, const std::shared_ptr<Surfel> &s2) {
                                return s1->id < s2->id;
                            });
                }), surfels.end());

        // optionally log
        if (properties.getBooleanProperty("log-dropped-surfels")) {
            cout << "Removed " << surfels_to_remove.size() << " of " << initial_surfel_count
                 << " surfels. Surfels remaining: " << surfels.size() << endl;
            for (const auto &id : surfels_to_remove) {
                cout << "  " << id << endl;
            }
        }
    }
}

/**
 * Remove Surfels in the provided list from the neighbours list of esch surfel in the surfels list.
 * @param surfels The list of source Surfels
 * @param surfels_to_remove The list of surfels to be removed from source Surfels' neighbours
 * @param properties Loggin settings
 */
void
prune_surfel_neighbours(std::vector<std::shared_ptr<Surfel>> &surfels,
                        std::vector<std::shared_ptr<Surfel>> &surfels_to_remove,
                        const Properties &properties) {
    using namespace std;

    // Now update lists of neighbours to remove ones that have gone
    for (auto &surfel : surfels) {
        remove_surfels_by_id(surfel->neighbouring_surfels, surfels_to_remove, properties);
    }
}

/**
 * For each Surfel in the current layer, find parent(s) and initialise this surfels
 * tangent with a combination of the parents tangents.
 * Surfels with no parents are pruned.
 * @param current_level_surfels
 * @param previous_level_surfels
 * @param properties
 */
void
initialise_tangents_from_previous_level(std::vector<std::shared_ptr<Surfel>> &current_level_surfels,
                                        const std::vector<std::shared_ptr<Surfel>> &previous_level_surfels,
                                        const Properties &properties) {
    using namespace std;

    vector<shared_ptr<Surfel>> orphans;
    multimap<shared_ptr<Surfel>, shared_ptr<Surfel>> child_to_parent_surfel_id_map = compute_child_to_parent_surfel_map(
            current_level_surfels,
            previous_level_surfels,
            orphans);
    // Remove orphan surfels from this level
    remove_surfels_by_id(current_level_surfels, orphans, properties);

    // Remove orphan surfels from neighbourhoods of current level surfels
    prune_surfel_neighbours(current_level_surfels, orphans, properties);

    // Finally, remove them from the lookup table
    unmap_surfels(orphans);

    // Seed the current level surfels with tangents from their parents.
    down_propagate_tangents(child_to_parent_surfel_id_map);

    // Remove previous level surfels
    unmap_surfels(previous_level_surfels);
}