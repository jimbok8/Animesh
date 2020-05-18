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
void PoSyOptimiser::optimise_surfel(std::shared_ptr<Surfel> surfel_ptr) {
}

std::vector<std::shared_ptr<Surfel>> PoSyOptimiser::select_surfels_to_optimise() {
    return std::vector<std::shared_ptr<Surfel>>{};
}

