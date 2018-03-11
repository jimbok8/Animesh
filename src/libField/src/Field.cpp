#include <Field/Field.h>
#include <Element/Element.h>
#include <VectorAngle/VectorAngle.h>

Field::~Field( ) {
	delete m_graph;
}


/**
 * Construct the field using a graph builder and some elements
 */
Field::Field( const GraphBuilder * const graph_builder, const std::vector<Element>& elements ) {
	m_graph = graph_builder->build_graph_for_elements( elements );
}


/**
 * @return the size of the ifled
 */
std::size_t Field::size() const {
	return m_graph->m_data_to_node_map.size();
}


/**
 * Smooth the field once, applying smoothing to each node
 * @return the largest error in tangent
 */
float Field::smooth_once( ) {
	using namespace Eigen;

	std::vector<Vector3f> new_tangents;

	// For each graphnode, compute the smoothed tangent
	for( auto map_iter = m_graph->m_data_to_node_map.begin(); map_iter != m_graph->m_data_to_node_map.end(); ++map_iter ) {
		GraphNode * g = (*map_iter).second;
		Vector3f v = get_smoothed_tangent_data_for_node( g );
		new_tangents.push_back( v );
	}

	// And then update the tangents
	float cost = 0.0f;
	auto tan_iter = new_tangents.begin();
	for( auto map_iter = m_graph->m_data_to_node_map.begin(); map_iter != m_graph->m_data_to_node_map.end(); ++map_iter, ++tan_iter ) {
		FieldData * fd = (FieldData *) (*map_iter).first;

		Vector3f current_tangent = fd->tangent();
		Vector3f new_tangent = (*tan_iter);
		float a = angleBetweenVectors( current_tangent, new_tangent ) - M_PI;
		cost += (a*a);
		fd->set_tangent( new_tangent );
	}

	return cost;
}


/**
 * Smooth node
 * Smooth a node in the field by averaging it's neighbours
 * @return The new vector.
 */
Eigen::Vector3f Field::get_smoothed_tangent_data_for_node( const GraphNode * const gn ) const {
	using namespace Eigen;

	FieldElement * this_fe = (FieldElement *) gn->m_data;

	Vector3f smoothed = this_fe->m_tangent;

	for( auto edge_iter = gn->m_edges.begin(); edge_iter != gn->m_edges.end(); ++edge_iter ) {

		const GraphNode * neighbouring_node = std::get<2>(*edge_iter);
		FieldElement * neighbour_fe = (FieldElement *) neighbouring_node->m_data;

		Vector3f best = best_rosy_vector_by_dot_product( 
			smoothed, 
			this_fe->m_normal,
			0, 
			neighbour_fe->m_tangent, 
			neighbour_fe->m_normal);

		smoothed = smoothed + best;

		smoothed = reproject_to_tangent_space( smoothed, this_fe->m_normal );
	}

	return smoothed;
}



/**
 * Given an arbitrary vector v, project it into the plane whose normal is given as n
 * also unitize it.
 * @param v The vector
 * @param n The normal
 * @return a unit vector in the tangent plane
 */
Eigen::Vector3f reproject_to_tangent_space( const Eigen::Vector3f& v, const Eigen::Vector3f& n) {
	using namespace Eigen;

	float error = v.dot( n );
	Vector3f projection = v - ( error * n );
	projection.normalize();
	return projection;
}


/**
 * Return vector of elements */
const std::vector<const FieldElement *> Field::elements( ) const {
	std::vector<const FieldElement *> elements;
	for( auto node_iter = m_graph->m_data_to_node_map.begin(); node_iter != m_graph->m_data_to_node_map.end(); ++node_iter ) {
		const FieldElement * fe = (FieldElement *) (*node_iter).first;

		elements.push_back( fe );
	}
	return elements;
}
