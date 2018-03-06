#include <Element/Element.h>

const float EPSILON = 1e-6;

Element::Element( const Eigen::Vector3f& location, const Eigen::Vector3f& normal ) {
	if( fabs(normal.norm() - 1.0f ) > EPSILON ) 
		throw std::invalid_argument( "Normal vector must be unit");

	m_location = location;
	m_normal= normal;
}