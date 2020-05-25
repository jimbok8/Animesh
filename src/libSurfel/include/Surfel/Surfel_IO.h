#pragma once

#include "Surfel.h"

#include <vector>
#include <string>
#include <memory>

/**
 * Save surfel data as binary file to disk
 */
void
save_surfels_to_file(const std::string& file_name,
                     const std::vector<std::shared_ptr<Surfel>>& surfels);

/**
 * Load surfel data from binary file
 */
void
load_from_file( const std::string& file_name,
                std::vector<std::shared_ptr<Surfel>>& surfels);

