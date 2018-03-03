#include <fstream>
#include <sstream>

#include <Graph/Graph.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

int main( int argc, char * argv[] ) {

	EdgeManager *em = new GridEdgeManager{1.0f};
	Graph *g = new Graph( em );

	for( int y = 0; y<10; y++ ) {
		for( int x = 0; x< 10; x++ ) {

			Vector3f location { x, y, 0.0f };
			Vector3f normal{ 0.0f, 0.0f, 1.0f };
			Element *e = new Element( location, normal );

			g->addElement( *e );
		}
	}

	Field field{ g };

	for( int i=0; i<10; i++ ) {
		field.smooth_once( );

		std::ostringstream oss;
		oss << "/Users/dave/Desktop/blob" << i << ".mat";
		std::ofstream file{ oss.str() };
		FieldExporter * fe = new MatlabFieldExporter( file );
		fe->exportField( field );

		delete fe;
	}
	
    return 0;
}
