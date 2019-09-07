#pragma once
#include "surfel_compute.h"

/**
 * Save surfel data as binary file to disk
 */
void
save_to_file( const std::string& file_name,
              const std::vector<Surfel>& surfels);

/**
 * Load surfel data from binary file
 */
void
load_from_file( const std::string& file_name,
                std::vector<Surfel>& surfels);

