#include <Graph/GraphNode.h>

#pragma once

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
	virtual void performEdgeManagement( GraphNode& newNode, std::vector<GraphNode> existingNodes ) = 0;
};