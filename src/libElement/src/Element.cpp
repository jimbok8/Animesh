#include <Element/Element.h>

Element::Element( const Eigen::Vector3f& location, const Eigen::Vector3f& normal ) {
	if( normal.norm() != 1.0f ) 
		throw std::invalid_argument( "Normal vector must be unit");

	m_location = location;
	m_normal= normal;
}