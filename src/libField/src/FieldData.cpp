#include <Field/Field.h>
#include <Eigen/Geometry>
#include <VectorAngle/VectorAngle.h>

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
Eigen::Vector3f best_rosy_vector_for( const Eigen::Vector3f& targetVector, 
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
		if( dp > bestDotProduct) {
			bestDotProduct = dp;
			best = testVector;
		}
	}
	return best;
}