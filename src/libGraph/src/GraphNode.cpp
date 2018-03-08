#include <Graph/GraphNode.h>

GraphNode::GraphNode( const Element& element ) : m_element{element} {

}

/**
 * @return the number of neighbours
 */
std::size_t GraphNode::num_neighbours() const {
	return m_edges.size();
}
