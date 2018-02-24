#pragma once

#include <Eigen/Core>
#include <Element/Element.h>


class FieldData {
public:
	FieldData( const Element& e );

	/**
	 * @return the tangent for this FieldData
	 */
	Eigen::Vector3f		tangent( ) const;

	/**
	 * @return the k value for this field data
	 */
	unsigned int		k( ) const;
	
private:
	/** The tangent vector to the surface at element e */
	Eigen::Vector3f		m_tangent;

	/** The k-rosy mutiple of 2pi/k for this field node */
	unsigned int		m_k;
};