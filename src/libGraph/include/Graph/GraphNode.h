#pragma once

#include <vector>
#include <Eigen/Core>
#include <Element/Element.h>

#include "Edge.h"


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
	 * @return the number of neighbours
	 */
	std::size_t num_neighbours() const;

	/** Iterator type for read only access to edges */
	typedef std::vector<Edge *>::const_iterator const_iterator;

	/** */
 	const_iterator begin() const { return m_edges.begin(); }

 	/** */
  	const_iterator end() const { return m_edges.end(); }


	/** Return the edges mutably until I can make EdgeManager a friedn  */
  	std::vector<Edge *>& 	edges( ) { return m_edges; } 

private:	
	/** The Element */
	Element						m_element;

	/** Neighbours of this node */
	std::vector<Edge *>		 	m_edges;
};