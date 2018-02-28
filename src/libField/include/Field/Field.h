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
	 * Smooth node
	 * Smooth a node in the field by averaging it's neighbours
	 * @return The new vector.
	 */
	Eigen::Vector3f get_smoothed_tangent_data(  GraphNode * gn );


private:
	/** The Graph - helps us get neghbours */
	const Graph * const 	m_graph;

	/** A Map from GraphNode to FieldData */
	std::unordered_map<const GraphNode *, FieldData *>	m_node_to_field_data_map;
};