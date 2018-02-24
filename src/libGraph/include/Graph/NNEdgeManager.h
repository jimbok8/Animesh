#include <Graph/EdgeManager.h>

class NNEdgeManager : EdgeManager {
public:
	NNEdgeManager( unsigned int max_neighbours ) : m_max_neighbours{ max_neighbours }{};
	
	/**
	 * Perform edge management
	 */
	virtual void performEdgeManagement( GraphNode * new_node, std::vector<GraphNode *>& existing_nodes ) const;

	/** 
	 * For a given node, if the node does not have sufficient neghbours (less than max_neighbours) or
	 * where new_node is closer to node than the furthest existing neghbour, add new_node to the list of
	 * neighbours, removing the furthest node if necessary.
	 * @param new_node The node to be added
	 * @param node_neighbours The list of pointers to nodes into which we insert new_node
	 * @param node The node from which distances are measured
	 */
	void manageEdgesFromNode( GraphNode * node, GraphNode * new_node ) const;

	/** 
	 * Insert new_node into node_neighbours in dustance from node order.
	 * @param new_node The node to be added
	 * @param node_neighbours The list of pointers to nodes into which we insert new_node
	 * @param node The node from which distances are measured
	 */
	void insertNodeInList( GraphNode * new_node, GraphNode * node ) const;

	/** The maximum number of neighbours per node */
	unsigned int m_max_neighbours;
};