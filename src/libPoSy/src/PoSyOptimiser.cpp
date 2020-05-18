//
// Created by Dave Durbin on 18/5/20.
//

#include "PoSyOptimiser.h"

#include <utility>
#include <vector>


// ========
/**
 * Construct a PoSyOptimiser.
 * @param properties Parameters for the optimiser.
 */
PoSyOptimiser::PoSyOptimiser(Properties properties) : AbstractOptimiser(std::move(properties)) {
}

PoSyOptimiser::~PoSyOptimiser() = default;


bool PoSyOptimiser::is_converged() {
    return true;
}

void PoSyOptimiser::optimisation_began() {

}

void PoSyOptimiser::optimisation_ended() {

}

void PoSyOptimiser::optimise_surfel(std::shared_ptr<Surfel> surfel_ptr) {

}

std::vector<std::shared_ptr<Surfel>> PoSyOptimiser::select_surfels_to_optimise() {
    return std::vector<std::shared_ptr<Surfel>>{};
}

