#include <Graph/Graph.h>

Graph::Graph( const EdgeManager * const edgeManager ) : mEdgeManager( edgeManager ){
}



std::size_t Graph::size() const {
	return mNodes.size();
}
