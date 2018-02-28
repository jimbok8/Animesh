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
 * Smooth the field repeatedly until the error diminishes below
 * a given theshold
 */

/**
 * Smooth the field once. 
 * Averages neghbouring tangents for each tangent node
 * @return The maximum value by which any node changed
 */

/**
 * Smooth node
 * Smooth a node in the field by averaging it's neighbours
 * @return The new vector.
 */
Eigen::Vector3f Field::get_smoothed_tangent_data( GraphNode * gn ) {
	using namespace Eigen;

	Vector3f	total{ 0.0f, 0.0f, 0.0f };
	int 		count = 0;

	FieldData * node_field_data = m_node_to_field_data_map.at( gn );

	for( auto g = gn->neighbours().begin(); g != gn->neighbours().end(); ++g ) {
		// Pick up field data
		FieldData * other_field_data = m_node_to_field_data_map.at( *g );

		// Get the best version given current node data
		Vector3f best = best_rosy_vector_for( node_field_data->tangent(), gn->element().normal(), 0, 
											  other_field_data->tangent(),(*g)->element().normal() );

		// Add into running total
		total += best;
		count++;
	}

	// Compute mean
	total /= count;
	return total;
}
