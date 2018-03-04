#include <Graph/GridEdgeManager.h>

const float EPSILON = 1e-6;

void GridEdgeManager::performEdgeManagement( GraphNode * new_node, std::vector<GraphNode *>& existing_nodes ) const {
	// For each node in existing_nodes
	for( auto & n : existing_nodes ) {
		manageEdgesFromNode( n, new_node);
	}
}
	
/** 
 * For a given node, if the new node is separated on x, y or z axies (single axis only) by the 
 * grid_spacing distance, then the new node is added as a neighbour
 * @param new_node The node to be added
 * @param node The node from which distances are measured
 */
void GridEdgeManager::manageEdgesFromNode( GraphNode * node, GraphNode * new_node ) const {
	// Get the neghbours of the node under consideration
	std::vector<GraphNode *>& current_neighbours = node->neighbours( );

	Eigen::Vector3f node_location = node->element().location();
	Eigen::Vector3f new_node_location = new_node->element().location();
	Eigen::Vector3f delta = node_location - new_node_location;


	if ( ( (fabs( fabs( delta[0] ) - m_grid_spacing)  < EPSILON ) 
		&& (fabs( delta[1] )                          < EPSILON ) 
		&& (fabs( delta[2] )                          < EPSILON ) ) ||

		 ( (fabs( delta[0] )                          < EPSILON ) 
		&& (fabs( fabs( delta[1] ) - m_grid_spacing)  < EPSILON ) 
		&& (fabs( delta[2] )                          < EPSILON ) ) ||

		 ( (fabs( delta[0] )                          < EPSILON ) 
		&& (fabs( delta[1] )                          < EPSILON ) 
		&& (fabs( fabs( delta[2] ) - m_grid_spacing)  < EPSILON ) ) ) {

		new_node->neighbours( ).push_back( node );
		node->neighbours( ).push_back( new_node );
	}
}
