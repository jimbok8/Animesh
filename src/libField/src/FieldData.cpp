#include <Field/Field.h>
#include <Eigen/Geometry>

FieldData::FieldData( const Element& e ) {
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

/**
 * @return k
 */
unsigned int FieldData::k( ) const {
	return m_k;
}
