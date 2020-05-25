#include <iostream>
#include <Eigen/Geometry>
#include <RoSy/RoSy.h>
#include <Geom/Geom.h>

const float EPSILON = 1e-4;

/**
* @param target_vector The vector we're trying to match.
* @param target_normal The normal about which to rotate it.
* @param source_vector The vector we're matching to the target.
* @param source_normal The normal about which to rotate it.
* @return the best fitting vector (i.e. best multiple of PI/2 + angle)
*/
std::pair<Eigen::Vector3f, Eigen::Vector3f>
best_rosy_vector_pair(const Eigen::Vector3f &target_vector, const Eigen::Vector3f &target_normal,
                      const Eigen::Vector3f &source_vector, const Eigen::Vector3f &source_normal) {
    int source_k = 0, target_k = 0;
    return best_rosy_vector_pair(target_vector, target_normal, target_k, source_vector, source_normal, source_k);
}

/**
* @param target_vector The vector we're trying to match.
* @param target_normal The normal about which to rotate it.
* @param target_k The number of rotations required for the match (output).
* @param source_vector The vector we're matching to the target.
* @param source_normal The normal about which to rotate it.
* @param source_k The number of rotations required for the match (output).
* @return the best fitting vector (i.e. best multiple of PI/2 + angle)
*/
std::pair<Eigen::Vector3f, Eigen::Vector3f>
best_rosy_vector_pair(const Eigen::Vector3f &target_vector, const Eigen::Vector3f &target_normal, int &target_k,
                      const Eigen::Vector3f &source_vector, const Eigen::Vector3f &source_normal, int &source_k) {
    using namespace Eigen;

    if (!is_unit_vector(source_normal)) {
        throw std::invalid_argument("Normal must be unit vector");
    }
    if (!is_unit_vector(target_normal)) {
        throw std::invalid_argument("Normal must be unit vector");
    }
    if (is_zero_vector(source_vector)) {
        throw std::invalid_argument("Vector may not be zero length");
    }
    if (is_zero_vector(target_vector)) {
        throw std::invalid_argument("Vector may not be zero length");
    }

    // We'll compare 0 and 90 degree rotations of each vector
    const Vector3f target_candidates[2] = {target_vector, target_normal.cross(target_vector)};
    const Vector3f source_candidates[2] = {source_vector, source_normal.cross(source_vector)};


    float best_dot_product = -std::numeric_limits<float>::infinity();;
    int best_target_idx = 0;
    int best_source_idx = 0;

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {

            float dp = std::abs(target_candidates[i].dot(source_candidates[j]));
            if (dp > best_dot_product) {
                best_dot_product = dp;
                best_target_idx = i;
                best_source_idx = j;
            }
        }
    }

    const float dp = target_candidates[best_target_idx].dot(source_candidates[best_source_idx]);
    target_k = best_target_idx;
    source_k = (dp >= 0.0f) ? best_source_idx : best_source_idx + 2;
    return std::make_pair(target_candidates[best_target_idx],
                          source_candidates[best_source_idx] * std::copysign(1.0f, dp));
}

/**
* Combine two tangent vectors with weighting
* @param v1 The first vector
* @param v2 The second vector
* @param n1 The first normal
* @param n2 The second normal
* @param w1 Weighting for the first vector
* @param w2 Weighting for the second vector
*/
Eigen::Vector3f average_rosy_vectors(const Eigen::Vector3f &v1,
                                     const Eigen::Vector3f &n1,
                                     float w1,
                                     const Eigen::Vector3f &v2,
                                     const Eigen::Vector3f &n2,
                                     float w2) {
    using namespace Eigen;

    // Find best matching rotation
    std::pair<Vector3f, Vector3f> result = best_rosy_vector_pair(v1, n1, v2, n2);
    Eigen::Vector3f v = (result.first * w1) + (result.second * w2);
    v = reproject_to_tangent_space(v, n1);
    return v;
}
