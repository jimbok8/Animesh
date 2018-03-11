#pragma once

#include <map>
#include <vector>

#include <Element/Element.h>

/**
 * A node in the graph
 */
struct GraphNode {
	// For each adjacent node we store cost, an edge payload and the node
    typedef std::tuple<float, void *, GraphNode*> 	Edge;		// weight, edge data, next node
    std::vector<Edge>					 			m_edges;
    const void * 									m_data;

    GraphNode( const void * data) : m_data{ data } {}
};

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
	 * Add a node with the given payload
	 * @param data THe data to be added. This should not exist in the graph already.
	 * We check this by doing a direct point comparison rather than a more sophisticated
	 * equality test
	 */
	GraphNode * add_node( const void * data );

	/**
	 * Add an edge to the graph connecting two existing nodes
	 */
	void add_edge( const void * from_data, const void * to_data, float weight, void * edge_data);

	/**  Map from data pointer to the node */
	typedef std::map<const void *, GraphNode *> DataNodeMap;
	DataNodeMap m_data_to_node_map;
};