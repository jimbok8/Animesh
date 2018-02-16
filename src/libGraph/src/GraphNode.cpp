#include <Graph/GraphNode.h>

GraphNode::GraphNode( const Eigen::Vector3f& point, const Eigen::Vector3f& normal ) {
	if( normal.norm() != 1.0f ) 
		throw std::invalid_argument( "Normal vector must be unit");

	mPoint = point;
	mNormal= normal;
}