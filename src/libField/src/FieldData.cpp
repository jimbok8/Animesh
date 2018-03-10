#include <Field/Field.h>
#include <Eigen/Geometry>
#include <VectorAngle/VectorAngle.h>
#include <iostream>

const float EPSILON = 1e-6;

FieldData::FieldData( const Element& e )  {
	// Construct a random tangent vector by:
	//    Construct random unit vector
	//    Find cross product with normal-> this is tangent
	using namespace Eigen;

	Vector3f random = VectorXf::Random(3);
	m_tangent = random.cross( e.normal( ) ).normalized( );

	// Set k to 0
	m_k = 0;
}

/**
 * @return the tangent for this FieldData
 */
Eigen::Vector3f FieldData::tangent( ) const {
	return m_tangent;
}

void FieldData::set_tangent( const Eigen::Vector3f& new_tangent ) {
	m_tangent = new_tangent;
}

/**
 * @return k
 */
unsigned int FieldData::k( ) const {
	return m_k;
}

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_by_dot_product( const Eigen::Vector3f& targetVector, 
									  const Eigen::Vector3f& targetNormal, 
									  int targetK, 
									  const Eigen::Vector3f& sourceVector, 
									  const Eigen::Vector3f& sourceNormal ) {
	using namespace Eigen;

	Vector3f effectiveTarget = vectorByRotatingOAroundN( targetVector, targetNormal, targetK );
	Vector3f best{ sourceVector };
	float bestDotProduct = effectiveTarget.dot( sourceVector );

	for( int k=1; k<4; ++k ) {
		Vector3f testVector = vectorByRotatingOAroundN( sourceVector, sourceNormal, k );

		float dp = effectiveTarget.dot( testVector );
		if( dp > bestDotProduct + EPSILON) {
			bestDotProduct = dp;
			best = testVector;
		}
	}
	return best;
}

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_for( const Eigen::Vector3f& targetVector, 
									  const Eigen::Vector3f& targetNormal, 
									  int targetK, 
									  const Eigen::Vector3f& sourceVector, 
									  const Eigen::Vector3f& sourceNormal ) {
	using namespace Eigen;

	Vector3f effectiveTarget = vectorByRotatingOAroundN( targetVector, targetNormal, targetK );
	Vector3f best{ sourceVector };
	float bestAngle = 2 * M_PI;

	for( int k=1; k<4; ++k ) {
		Vector3f testVector = vectorByRotatingOAroundN( sourceVector, sourceNormal, k );

		float theta = angleBetweenVectors( effectiveTarget, testVector ) - M_PI;
		if( fabsf(theta) < fabsf(bestAngle) ) {
			bestAngle = theta;
			best = testVector;
		}
	}
	return best;
}

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_and_kl( const Eigen::Vector3f& target_vector, const Eigen::Vector3f& target_normal, int& k_ij, 
									  	 const Eigen::Vector3f& source_vector, const Eigen::Vector3f& source_normal, int& k_ji ) {
	using namespace Eigen;

	float best_dot_product	= -2.0f;
	Vector3f best_vector{ 0.0f, 0.0f, 0.0f };

	int k =0, l=0;
	for( k =0; k < 4; ++k ) {

		Vector3f test_target = vectorByRotatingOAroundN( target_vector, target_normal, k );

		for( l =0; l < 4; ++l ) {
			Vector3f test_source = vectorByRotatingOAroundN( source_vector, source_normal, l );

			float dp = test_target.dot( test_source );
			// Force a new solution to be better than an existing one
			if( dp > (best_dot_product + EPSILON) ) {
				best_dot_product = dp;
				best_vector = test_source;
			}
		}
	}

	k_ij = k;
	k_ji = l;
	return best_vector;
}