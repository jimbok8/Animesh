#pragma once 

#include <unordered_map>

#include "FieldData.h"
#include <Graph/Graph.h>
#include <unordered_map>

class Field {
public:
	/**
	 * Construct the field for an initial Graph
	 * This method will create a map from location to FieldData for each location in the Graph.
	 */
	Field( const Graph * const graph );

	~Field( );

	/** Access the index'th node of the field
	*/
	const std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> dataForGraphNode( unsigned int index ) const;

	/**
	 * @return the size of the ifled
	 */
	unsigned int size() const;


	/**
	 * Smooth the field once, applying smoothing to each node
	 * @return the largest error in tangent
	 */
	float smooth_once( );

	/**
	 * Smooth node
	 * Smooth a node in the field by averaging it's neighbours
	 * @return The new vector.
	 */
	Eigen::Vector3f get_smoothed_tangent_data_for_node( const GraphNode * const gn ) const;


private:
	/** The Graph - helps us get neghbours */
	const Graph * const 	m_graph;

	/** A Map from GraphNode to FieldData */
	std::unordered_map<const GraphNode *, FieldData *>	m_node_to_field_data_map;
};

/**
 * Given an arbitrary vector v, project it into the plane whose normal is given as n
 * also unitize it.
 * @param v The vector
 * @param n The normal
 * @return a unit vector in the tangent plane
 */
Eigen::Vector3f reproject_to_tangent_space( const Eigen::Vector3f& v, const Eigen::Vector3f& n);
