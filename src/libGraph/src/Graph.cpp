#include <Graph/Graph.h>

Graph::Graph( const Eigen::Matrix<float, 3, Eigen::Dynamic>& points, const Eigen::Matrix<float, 3, Eigen::Dynamic>& normals, Eigen::MatrixXi& neighbours ) {
	using namespace Eigen;

	if( points.cols() != normals.cols() )
		throw std::invalid_argument( "Points and normals must have the same dimensions");

	if( ( points.cols() != neighbours.cols() ) || ( points.cols() != neighbours.rows() ) )
		throw std::invalid_argument( "Neighbours has incorrect dimensions");

	// For each point, construct a node with the appropriate point and normal
	size_t numPoints = points.cols();
	for( size_t i=0; i<numPoints; i++ ) {
		Vector3f point = points.col(i);
		Vector3f normal = normals.col(i);
		mNodes.push_back( GraphNode{ point, normal });
	}


	// Now iterate over the neighbour matrix to add references to the GraphNodes
}



std::size_t Graph::size() {
	return mNodes.size();
}
