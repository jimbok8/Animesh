#pragma once


class GraphNode;
/*
 * Edge is a directed edge on a graph. Each node has a collection of edges. An edge
 * stores a pointer to the destination node, source node and has a weight.
 * It may also store a templated type.
 */

class Edge {
public:
	/**
	 * Make a GraphNode given an Element
	 * @param element The Element to store in the node
	 */
	Edge( const GraphNode * const source_node, 
	      const GraphNode * const dest_node,
	      float					  weight,
	      const void *			  data ) : m_source_node{ source_node }, m_dest_node { dest_node}, m_weight{ weight }, m_data{ data } {}

	/**
	 * @return the data for this node
	 */
	const void * data() const { return m_data; };

	/**
	 * @return the weight of this edge
	 */
	float weight( ) const { return m_weight; };

	/**
	 * @return the source_node
	 */
	const GraphNode * const source_node() const { return m_source_node; }

	/**
	 * @return the dest_node
	 */
	const GraphNode * const dest_node() const { return m_dest_node; }
	

private:	
	/** The Element */
	const void *			m_data;

	const GraphNode			* const m_source_node;

	const GraphNode			* const m_dest_node;

	float					m_weight;
};