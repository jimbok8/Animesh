#include <Eigen/Core>
#include <Graph/GraphNode.h>
#include <Graph/EdgeManager.h>
#include <vector>
#include <unordered_map>

#pragma once

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
	 * Construct a Graph with no nodes
	 * @param edgeManager A class responsible for updating the edges of a graph as new Elements are added
	 */
	Graph( const EdgeManager * const edgeManager );

	/**
	 * Add an element to the Graph, updating neighbourhoods accordingly
	 */
	void addElement( const Element& element );

	/**
	 * @return the size of the Graph which is the number of GraphNodes
	 */
	std::size_t size() const;


private:
	/** The nodes for this graph */
	std::vector<GraphNode> 	mNodes;

	/** The EdgeManager */
	const EdgeManager 			* const mEdgeManager;


	/** Neighbours of nodes */
	//std::unordered_map<GraphNode, std::vector<GraphNode *>> mNeighbours;
};