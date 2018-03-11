#pragma once

#include "GraphBuilder.h"

class NearestNeighbourGraphBuilder : public GraphBuilder {
public:
	NearestNeighbourGraphBuilder( unsigned int max_neighbours ) : m_max_neighbours{ max_neighbours }{};

	Graph * build_graph_for_elements( const std::vector<Element>& elements ) const;

	void insert_edge_if_appropriate( Graph * graph, GraphNode * from, GraphNode * to ) const;

	void sort_edges( FieldElement * fe_from, std::vector<GraphNode::Edge>& edges ) const;

	/** The maximum number of neighbours per node */
	unsigned int m_max_neighbours;
};

float distance_between( const FieldElement * fe1, const FieldElement * fe2 );
