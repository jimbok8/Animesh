#include <Graph/Graph.h>

void Graph::add_node ( const void * data  ) {
    DataNodeMap::iterator itr = m_data_to_node_map.find(data);
    if (itr == m_data_to_node_map.end()) {
        GraphNode * gn = new GraphNode( data );
        m_data_to_node_map[data] = gn;
        return;
    }
    throw std::invalid_argument( "GraphNode already exists" );
}

/**
 * Add an edge to the graph connecting two existing nodes
 */
void Graph::add_edge( const void * from_data, const void * to_data, float weight, void * edge_data) {
    using namespace std;

    GraphNode * from_node  = m_data_to_node_map.find(from_data)->second;
    GraphNode * to_node    = m_data_to_node_map.find(to_data)->second;
    GraphNode::Edge edge   = make_tuple(weight, edge_data, to_node);
    from_node->m_edges.push_back(edge);
}
