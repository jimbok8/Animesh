//
// Created by Dave Durbin on 26/3/20.
//

#ifndef ANIMESH_HIERARCHICAL_MESHER_UTILITIES_H
#define ANIMESH_HIERARCHICAL_MESHER_UTILITIES_H

#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <Properties/Properties.h>
#include <DepthMap/DepthMap.h>
#include <Surfel/Surfel.h>
#include <Surfel/PixelInFrame.h>
#include <Surfel/Surfel_Compute.h>
#include <Surfel/Surfel_IO.h>
#include <omp.h>
#include "correspondences_io.h"
#include "optimise.h"
#include "types.h"
#include "utilities.h"


/**
 * Create a map to allow lookup of a Surfel ID from one of its PIF
 */
std::map<PixelInFrame, std::shared_ptr<Surfel>>
map_pifs_to_surfel(const std::vector<std::shared_ptr<Surfel>> &surfels);

/**
 * Given a set of parent surfels and child surfels, estalish a mapping from child to one or more parents.
 * Children with no parents are stored as IDs in unmapped
 */
std::multimap<std::string, std::string>
compute_child_to_parent_surfel_map(const std::vector<std::shared_ptr<Surfel>> &child_surfels, //
                                   const std::vector<std::shared_ptr<Surfel>> &parent_surfels, //
                                   std::vector<std::shared_ptr<Surfel>> &orphans);

/**
 * Initialise the child surfel tangents from their psarwent surfel tangents
 * where the parent-child mappings are defined in child_to_parents
 */
void
down_propagate_tangents(const std::multimap<std::shared_ptr<Surfel>,
        std::shared_ptr<Surfel>> &child_to_parents);

std::vector<std::vector<PixelInFrame>>
get_correspondences(const Properties &properties,
                    unsigned int level,
                    const std::vector<DepthMap> &depth_map,
                    std::vector<Camera> &cameras);

void
maybe_save_depth_and_normal_maps(const Properties &properties,
                                 const std::vector<std::vector<DepthMap>> &depth_map_hierarchy);

/**
 * Remove any surfels which cannot be found from the neighbours of remaining surfels
 */
void
prune_surfel_neighbours(std::vector<std::shared_ptr<Surfel>> &surfels,
                        std::vector<std::shared_ptr<Surfel>> &surfels_to_remove,
                        const Properties &properties);

/**
 * Remove the previous level surfels from the Surfel::map
 */
void
unmap_surfels(const std::vector<std::shared_ptr<Surfel>> &surfel_ids);

/**
 * Remove the previous level surfels from the Surfel::map
 */
void
unmap_surfels(const std::vector<std::shared_ptr<Surfel>> &surfels);

/**
 * Given a list of surfel IDs, remove them from the surfel list.
 * The properties object is consulted to check whether to log this removal or not.
 */
void
remove_surfels_by_id(std::vector<std::shared_ptr<Surfel>> &surfels,
                     std::vector<std::shared_ptr<Surfel>> &surfels_to_remove,
                     const Properties &properties);


/**
 * For each Surfel in the current layer, find parent(s) and initialise this surfels
 * tangent with a combination of the parents tangents.
 * Surfels with no parents are pruned.
 * @param current_level_surfels
 * @param previous_level_surfels
 * @param properties
 */
void initialise_tangents_from_previous_level(std::vector<std::shared_ptr<Surfel>> &current_level_surfels,
                                             const std::vector<std::shared_ptr<Surfel>> &previous_level_surfels,
                                             const Properties &properties);

#endif //ANIMESH_HIERARCHICAL_MESHER_UTILITIES_H
