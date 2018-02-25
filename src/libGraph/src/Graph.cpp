#include <Graph/Graph.h>

Graph::Graph( const EdgeManager * const edge_manager ) : m_edge_manager{ edge_manager } {
    if( edge_manager == nullptr ) 
        throw std::invalid_argument{ "EdgeManager may not be null" };
}

Graph::~Graph( ) {
	// Delete all GraphNodes
	for( auto iter = m_nodes.begin(); iter != m_nodes.end(); ++iter ) {
		delete (*iter);
	}
}

void Graph::addElement( const Element& element ) {
    // Make a new GraphNode
    GraphNode * graph_node = new GraphNode{ element };

    // Use the edge manager to update the graph
    m_edge_manager->performEdgeManagement( graph_node, m_nodes );

    // Add the new node to the graph
    m_nodes.push_back( graph_node );
}


std::size_t Graph::size() const {
	return m_nodes.size();
}

/** 
 * @return the index'th node of the graph
 */
const GraphNode * Graph::node( unsigned int index ) const {
    if( index >= m_nodes.size() )
        throw std::invalid_argument{ "Index out of range" };

    return m_nodes[index];
}
