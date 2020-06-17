#pragma once

#include "Surfel.h"
#include <Graph/Graph.h>
#include <vector>
#include <string>
#include <memory>

/**
 * Save surfel data as binary file to disk
 */
void
save_surfel_graph_to_file(const std::string& file_name,
                          const animesh::Graph<std::shared_ptr<Surfel>, float>& surfel_graph);

/**
 * Load surfel data from binary file
 */
animesh::Graph<std::shared_ptr<Surfel>,float>
load_surfel_graph_from_file(const std::string &file_name);

