#include <Graph/NNEdgeManager.h>

void NNEdgeManager::performEdgeManagement( GraphNode * new_node, std::vector<GraphNode *>& existing_nodes ) const {
	// For each node in existing_nodes
	for( auto node : existing_nodes ) {
		manageEdgesFromNode( node, new_node);
	}
}
	
/** 
 * For a given node, if the node does not have sufficient neghbours (less than max_neighbours) or
 * where new_node is closer to node than the furthest existing neghbour, add new_node to the list of
 * neighbours, removing the furthest node if necessary.
 * @param new_node The node to be added
 * @param node The node from which distances are measured
 */
void NNEdgeManager::manageEdgesFromNode( GraphNode * node, GraphNode * new_node ) const {
	// Get the neghbours of the node under consideration
	std::vector<Edge *>& current_edges = node->edges( );

	bool inserted = false;

	// If this node does not yet have 'enough' neghbours, just add this in place
	if( current_edges.size() < m_max_neighbours ) {
		insertNodeInList( new_node, node );
		inserted = true;
	}

	// Otherwise, check that this node is close enough to replace an existing neighbour
	else {

		// Compute distance to existing node
		float distance_to_new_node = (node->element().location() - new_node->element().location()).norm();
		float distance_to_furthest_neighbour = (node->element().location() - current_edges.back()->dest_node()->element().location()).norm();

		if( distance_to_furthest_neighbour > distance_to_new_node ) {
			current_edges.erase( current_edges.end() - 1 );
			insertNodeInList( new_node, node );
			inserted = true;
		}
	}

	// If we inserted into the list, we ought to insert the other node into the new_node
	if( inserted ) {
		insertNodeInList( node, new_node );
		if( new_node->edges().size() > m_max_neighbours ) {
			new_node->edges().erase( new_node->edges().end() - 1 );
		}
	}
}

/** 
 * Insert new_node into node_neighbours in distance from node order.
 * @param new_node The node to be added
 * @param target_node The node into whose neghbours new_node should be inserted
 */
void NNEdgeManager::insertNodeInList( GraphNode * new_node, GraphNode * target_node ) const {
	Eigen::Vector3f target_location = target_node->element().location();

	// Insert the thing as close to the back of the list as it's distance from node warrants
	float distance_to_new_node = ( new_node->element().location() - target_location ).norm();

	auto edge_iter = target_node->edges( ).begin();
	while ( edge_iter != target_node->edges( ).end() ) { 
		Edge * current_edge = *edge_iter;
		const GraphNode * neighbour = current_edge->dest_node();

		Eigen::Vector3f current_neighbour_location = neighbour->element( ).location( );

		float distance_to_current_neghbour = ( current_neighbour_location - target_location).norm();
		if( distance_to_new_node < distance_to_current_neghbour ) {
			break;
		} else {
			++edge_iter;
		}
	}
	target_node->edges( ).insert( edge_iter, new Edge( target_node, new_node, 1.0f, nullptr ) );
}