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
	 * @return the tangent for this FieldData
	 */
	void				set_tangent( const Eigen::Vector3f& new_tangent );

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


/**
 * @param targetVector The vector we're trying to match
 * @param targetK The value of K for the target vector which should be locked
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_for( const Eigen::Vector3f& targetVector, 
									  const Eigen::Vector3f& targetNormal, 
									  int targetK, 
									  const Eigen::Vector3f& sourceVector, 
									  const Eigen::Vector3f& sourceNormal );

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
									  const Eigen::Vector3f& sourceNormal );

