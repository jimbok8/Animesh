#pragma once

#include "GraphNode.h"

/*
 * EdgeManager is responsible for updating the lcoations of Edges in a graph when 
 * new elements are added
 */
class EdgeManager {
public:
	/**
	 * Virtual destructur
	 */
	virtual ~EdgeManager( ) {};

	/**
	 * Perform edge management
	 */
	virtual void performEdgeManagement( GraphNode * new_node, std::vector<GraphNode *>& existingNodes ) const = 0;
};
