#pragma once

#include <Graph/EdgeManager.h>

/*
 * EdgeManager is responsible for updating the lcoations of Edges in a graph when 
 * new elements are added
 */
class GridEdgeManager : public EdgeManager {
public:
	GridEdgeManager( float grid_spacing ) : m_grid_spacing{ grid_spacing }{}

	/**
	 * Perform edge management
	 */
	virtual void performEdgeManagement( GraphNode * new_node, std::vector<GraphNode *>& existing_nodes ) const;

	void manageEdgesFromNode( GraphNode * node, GraphNode * new_node ) const;

private:

	float 			m_grid_spacing;
};
