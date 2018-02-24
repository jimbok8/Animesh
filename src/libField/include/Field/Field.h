#pragma once 

#include "FieldData.h"
#include <Graph/Graph.h>

class Field {
public:
	/**
	 * Construct the field for an initial Graph
	 * This method will create a map from location to FieldData for each location in the Graph.
	 */
	Field( const Graph * const graph );


private:
	/** The Graph - helps us get neghbours */
	const Graph * const 	m_graph;

	/** A Map from GraphNode to FieldData */
	std::unordered_map<GraphNode *, FieldData *>	m_node_to_field_data_map;
};