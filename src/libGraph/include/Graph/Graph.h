#include <Eigen/Core>
#include <Graph/GraphNode.h>


/*
 * Graph is graph representing a 3D structure (point cloud, mesh, triangle soup.
 * Each element of the structure is represnted as a GraphNode which stores its location
 * and normal.
 * The Graph stores the relationship between GraphNodes and can return the neghbours of
 * an individual node.
 */

class Graph {
public:
	/**
	 * Construct a Graph given an array of 3D points, their normals and their neighbours.
	 * @param points A 3xN matrix where N is the number of points and the ith column is the 3D location of that point
	 * @param normals A 3xN matrix where the ith column is the normal for the ith point
	 * @param neighbours An NxN sparse matrix with the entry at (i,j) being 1 if points i and j are neighbours otherwise 0.
	 */
	Graph( const Eigen::Matrix<float, 3, Eigen::Dynamic>& points, const Eigen::Matrix<float, 3, Eigen::Dynamic>& normals, Eigen::MatrixXf& neighbours );
};