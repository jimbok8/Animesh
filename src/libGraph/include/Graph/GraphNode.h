#include <Eigen/Core>
#include <Element/Element.h>

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
	const Element& element() const { return mElement; };


private:	
	/** The Element */
	Element mElement;
};