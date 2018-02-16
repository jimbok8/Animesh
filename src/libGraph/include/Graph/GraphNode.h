#include <Eigen/Core>

/*
 * GraphNode is a node in a Graph. It stores the 3D location of the element
 * and it's unit normal vector
 */

class GraphNode {
public:
	/**
	 * Make a GraphNode given an existing point and normal
	 * @param point The point
	 * @param normak The normal (assumed to be a unit vector );
	 */
	GraphNode( const Eigen::Vector3f& point, const Eigen::Vector3f& normal );

	/**
	 * @return the normal for this node
	 */
	const Eigen::Vector3f& normal() const {return mNormal; }

	/**
	 * @return the point for this node
	 */
	const Eigen::Vector3f& point() const {return mPoint; };


private:	
	/** The point in space */
	Eigen::Vector3f mPoint;

	/** The normal to the point */
	Eigen::Vector3f mNormal;
};