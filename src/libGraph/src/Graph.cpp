#include <Graph/Graph.h>

Graph::Graph( const EdgeManager * const edgeManager ) : mEdgeManager{ edgeManager } {
    if( edgeManager == nullptr ) 
        throw std::invalid_argument{ "EdgeManager may not be null" };
}


void Graph::addElement( const Element& element ) {
    // Make a new GraphNode
    GraphNode * graph_node = new GraphNode{ element };

    // Use the edge manager to update the graph
    mEdgeManager->performEdgeManagement( graph_node, mNodes );

    // Add the new node to the graph
    mNodes.push_back( graph_node );
}


std::size_t Graph::size() const {
	return mNodes.size();
}
