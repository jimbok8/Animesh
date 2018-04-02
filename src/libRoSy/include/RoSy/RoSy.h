#pragma once

#include <Eigen/Core>

/**
 * Compute the angle between two vectors
 */
float angle_between_vectors( Eigen::Vector3f v1, Eigen::Vector3f v2 );

/**
 * Compute the vector v resulting from the rotation of vector o around normal n
 * through 2*pi*k/4
 *
 * @param o The vector to be rotated
 * @param n The normal around which to rotate o
 * @param k The integral multiplier rotation (0-3)
 * @return The rotated vector 
 */
 Eigen::Vector3f vector_by_rotating_around_n( const Eigen::Vector3f & o, const Eigen::Vector3f & n, int k);

/**
 * Given an arbitrary vector v, project it into the plane whose normal is given as n
 * also unitize it.
 * @param v The vector
 * @param n The normal
 * @return a unit vector in the tangent plane
 */
Eigen::Vector3f reproject_to_tangent_space( const Eigen::Vector3f& v, const Eigen::Vector3f& n);


/**
 * @param targetVector The vector we're trying to match
 * @param targetK The value of K for the target vector which should be locked
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_for( const Eigen::Vector3f& target_vector, 
									  const Eigen::Vector3f& target_normal, 
									  const Eigen::Vector3f& source_vector, 
									  const Eigen::Vector3f& source_normal,
									  int& k_ij );

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_dp_rosy_vector_for( const Eigen::Vector3f& target_vector, 
										 const Eigen::Vector3f& target_normal, 
										 const Eigen::Vector3f& source_vector, 
										 const Eigen::Vector3f& source_normal,
										 int& k_ij );

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
void best_rosy_vector_and_kl( const Eigen::Vector3f& target_vector, const Eigen::Vector3f& target_normal, Eigen::Vector3f& best_target, int& k_ij, 
						  	  const Eigen::Vector3f& source_vector, const Eigen::Vector3f& source_normal, Eigen::Vector3f& best_source, int& k_ji );

