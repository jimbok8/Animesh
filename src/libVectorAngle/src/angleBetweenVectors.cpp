#include <iostream>
#include <Eigen/Geometry>
#include <VectorAngle/VectorAngle.h>

const float EPSILON = 1e-6;

/**
 * Compute the angle between two vectors
 */
float angleBetweenVectors( Eigen::Vector3f v1, Eigen::Vector3f v2 ) {
	if( (v1[0] == 0.0f && v1[1] == 0.0f && v1[2] == 0.0f ) ||
		(v2[0] == 0.0f && v2[1] == 0.0f && v2[2] == 0.0f ) ) {
		throw std::invalid_argument( "Vector may not be zero length" );
	}

	// Compute the angle between the vectors using 
	// θ=2 atan2(|| ||v||u−||u||v ||, || ||v||u+||u||v ||)
	Eigen::Vector3f vu = v1.norm() * v2;
	Eigen::Vector3f uv = v2.norm() * v1;
	float theta = 2 * atan2( (vu - uv).norm(), (vu + uv).norm() );	

	return theta;
}

/**
 * Compute the vector v resulting from the rotation of vector o around normal n
 * through 2*pi*k/4
 *
 * @param o The vector to be rotated
 * @param n The normal around which to rotate o
 * @param k The integral multiplier rotation (0-3)
 * @return The rotated vector 
 */
 Eigen::Vector3f vectorByRotatingOAroundN( const Eigen::Vector3f & o, const Eigen::Vector3f & n, int k) {
 	using namespace Eigen;

 	if( fabs( o.norm() ) < EPSILON )
 		throw std::invalid_argument( "Vector may not be zero length" );
 	
 	if( fabs( n.norm() - 1.0f ) > EPSILON )
 		throw std::invalid_argument( "Normal must be unit vector" );

 	float angle = (M_PI * 2 * k) / 4.0f;
 	AngleAxis<float> aa{ angle , n };
 	Quaternionf q{ aa };

	return q * o;
 } 

 /**
  * Compute the optimal k and l to minimise the angle between the two vectors o1 and o2 when rotated around n1 and n2
  * @param o1 The first tangent vector
  * @param n1 The first normal vector
  * @param o2 The second tangent vector - must be unit vector
  * @param n2 The second normal vector - must be unit vector
  * @param k The optimal rotation multiplier for o1/n1
  * @param l The optimal rotation multiplier for o2/n2
  */
 float computeOptimalKL( const Eigen::Vector3f& o1, const Eigen::Vector3f& n1, const Eigen::Vector3f& o2, const Eigen::Vector3f& n2, int& k, int& l) {
 	using namespace Eigen;

 	if( n1.norm() != 1.0f || n2.norm( ) != 1.0f )
 		throw std::invalid_argument( "Normal must be unit vector" );

 	if( o1.norm() == 0.0f || o2.norm() == 0.0f )
 		throw std::invalid_argument( "Vector may not be zero length" );

 	int bestI = -1;
 	int bestJ = -1;
 	float bestAngle = M_PI * 2.0f;

 	for( int i=0; i<4; i++ ) {
 		Vector3f roti = vectorByRotatingOAroundN(o1, n1, i);
 		for( int j = 0; j<4 ; j++ ) {
 			Vector3f rotj = vectorByRotatingOAroundN(o2, n2, j);

 			float angle = angleBetweenVectors(roti, rotj);
 			
 			if( angle < (bestAngle - (M_PI/180.0f) ) ) {

 				
 				bestAngle = angle;
 				bestI = i;
 				bestJ = j;
 			}
 		}
 	}

 	k = bestI;
 	l = bestJ;

 	return bestAngle;
 }
