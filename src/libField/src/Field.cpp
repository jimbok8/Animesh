#include <Field/Field.h>
#include <Element/Element.h>

Field::~Field( ) {
	// Delete the field data objects  created
	for( auto iter = m_node_to_field_data_map.begin(); iter != m_node_to_field_data_map.end(); ++iter ) {
		FieldData * value = iter->second;
		delete value;
	}
}


Field::Field( const Graph * const graph  ) : m_graph{ graph } {

	for( auto iter = graph->begin( ); iter != graph->end(); ++iter ) {
		GraphNode * gn = (*iter);

		FieldData * fd = new FieldData( gn->element( ) );

		m_node_to_field_data_map.insert( { gn, fd } );
	}
}


/**
 * @return the size of the ifled
 */
unsigned int Field::size() const {
	return m_node_to_field_data_map.size();
}


/** Access the index'th node of the field
*/
const std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> Field::dataForGraphNode( unsigned int index ) const {
	if( index >= m_node_to_field_data_map.size() )
		throw std::invalid_argument{ "Index out of range" };


	const GraphNode * gn = m_graph->node(index);
	FieldData * fd = m_node_to_field_data_map.at( gn );

	return std::make_tuple(  gn->element().location(), gn->element().normal(), fd->tangent() );
}

/**
 * Smooth node
 * Smooth a node in the field by averaging it's neighbours
 * @return The new vector.
 */
Eigen::Vector3f Field::get_smoothed_tangent_data_for_node( const GraphNode * const gn ) const {
	using namespace Eigen;

	std::vector<Vector3f> neighbour_tangents = best_rosy_vectors_for_neighbours_of_node( gn );

	Vector3f mean = compute_mean_vector( neighbour_tangents );

	// Normalise
	mean.normalize();


	return mean;
}


/**
 * Get a vectpr of the best fit vectors from the neighbours of a specific 
 * node in the graph.
 * @return the vector of tangents
 */
std::vector<Eigen::Vector3f> Field::best_rosy_vectors_for_neighbours_of_node( const GraphNode * const gn ) const {
	using namespace Eigen;

	std::vector<Vector3f> neighbours;

	FieldData * node_field_data = m_node_to_field_data_map.at( gn );

	for( auto g = gn->begin(); g != gn->end(); ++g ) {
		// Pick up field data
		FieldData * other_field_data = m_node_to_field_data_map.at( *g );

		// Get the best version given current node data
		Vector3f best = best_rosy_vector_for( node_field_data->tangent(), gn->element().normal(), 0, 
											  other_field_data->tangent(),(*g)->element().normal() );

		neighbours.push_back( best );
	}
	return neighbours;
}


/**
 * Smooth the field repeatedly until the error diminishes below
 * a given theshold
 */
// TODO

/**
 * Smooth the field once, applying smoothing to each node
 * @return the largest error in tangent
 */
float Field::smooth_once( ) {
	using namespace Eigen;

	std::vector<Vector3f> new_tangents;

	// For each graphnode, compute the smoothed tangent
	for( auto gn = m_graph->begin(); gn != m_graph->end(); ++gn ) {
		GraphNode * g = (*gn);
		Vector3f v = get_smoothed_tangent_data_for_node( g );
		new_tangents.push_back( v );
	}

	// And then update the tangents
	auto vn = new_tangents.begin();
	for( auto gn = m_graph->begin(); gn != m_graph->end(); ++gn, ++vn ) {
		FieldData * fd = m_node_to_field_data_map.at( (*gn) );
		fd->set_tangent( (*vn) );
	}

	return 0.0f;
}



/**
 * Compute the average of a set of tangent vectors
 * 
 * @return The average vector.
 */
Eigen::Vector3f compute_mean_vector( const std::vector<Eigen::Vector3f>& sourceVectors ) {

	if( sourceVectors.size()== 0 ) 
		throw std::invalid_argument{ "Vector can't be empty" };

	using namespace Eigen;

	Vector3f	total{ 0.0f, 0.0f, 0.0f };
	int 		count = sourceVectors.size();

	for( auto vi = sourceVectors.begin(); vi != sourceVectors.end(); ++vi ) {
		total += (*vi);
	}

	// Compute mean
	total /= count;
	return total;
}

