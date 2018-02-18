#include <Eigen/Core>

#pragma once

/*
 * Element represents any component that can have a location in 3D space, a normal and 
 * supports a cross field tangent.
 */

class Element {
public:
	/**
	 * Make an Element given an existing point and normal
	 * @param point The location
	 * @param normal The normal (msut be a unit vector);
	 */
	Element( const Eigen::Vector3f& location, const Eigen::Vector3f& normal );

	/**
	 * @return the normal for this Element
	 */
	const Eigen::Vector3f& normal() const {return mNormal; }

	/**
	 * @return the location for this Element
	 */
	const Eigen::Vector3f& location() const {return mLocation; };


private:	
	/** The location in space */
	Eigen::Vector3f mLocation;

	/** The normal to the Element */
	Eigen::Vector3f mNormal;
};