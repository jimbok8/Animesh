#include <Eigen/Core>

/**
 * Compute the angle between two vectors
 */
float angleBetweenVectors( Eigen::Vector3f v1, Eigen::Vector3f v2 );

/**
 * Compute the vector v resulting from the rotation of vector o around normal n
 * through 2*pi*k/4
 *
 * @param o The vector to be rotated
 * @param n The normal around which to rotate o
 * @param k The integral multiplier rotation (0-3)
 * @return The rotated vector 
 */
 Eigen::Vector3f vectorByRotatingOAroundN( const Eigen::Vector3f & o, const Eigen::Vector3f & n, int k);

 /**
  * Compute the optimal k and l to minimise the angle between the two vectors o1 and o2 when rotated around n1 and n2
  * @param o1 The first tangent vector
  * @param n1 The first normal vector
  * @param o2 The second tangent vector - must be unit vector
  * @param n2 The second normal vector - must be unit vector
  * @param k The optimal rotation multiplier for o1/n1
  * @param The optimal rotation multiplier for o2/n2
  */
 float computeOptimalKL( const Eigen::Vector3f& o1, const Eigen::Vector3f& n1, const Eigen::Vector3f& o2, const Eigen::Vector3f& n2, int& k, int& l);

