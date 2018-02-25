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