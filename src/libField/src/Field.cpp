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
