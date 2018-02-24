#include <Eigen/Core>
#include <Element/Element.h>
#include <vector>

#pragma once

/*
 * GraphNode is a node in a Graph. It stores the 3D location of the element
 * and it's unit normal vector
 */

class GraphNode {
public:
	/**
	 * Make a GraphNode given an Element
	 * @param element The Element to store in the node
	 */
	GraphNode( const Element& element );

	/**
	 * @return the Element for this node
	 */
	const Element& element() const { return m_element; };

	/**
	 * @return the neighbours of this GraphNode
	 */
	std::vector<GraphNode *>& 	neighbours()  { return m_neighbours; };


private:	
	/** The Element */
	Element						m_element;

	/** Neighbours of this node */
	std::vector<GraphNode *> 	m_neighbours;
};