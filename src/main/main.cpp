#include <fstream>
#include <sstream>

#include <Graph/Graph.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

const int DIM = 10;
const int FRAMES = 10;

int main( int argc, char * argv[] ) {


	EdgeManager *em = new GridEdgeManager{1.0f};
	Graph *g = new Graph( em );

	for( int y = 0; y<DIM; y++ ) {
		for( int x = 0; x<DIM; x++ ) {
			Vector3f location { x, y, 0.0f };
			Vector3f normal{ 0.0f, 0.0f, 1.0f };
			Element *e = new Element( location, normal );

			g->addElement( *e );
		}
	}

	Field field{ g };


	std::ofstream initial_file{ "/Users/dave/Desktop/initial.mat" };
	FieldExporter * fe = new MatlabFieldExporter( initial_file );
	fe->exportField( field );
	delete fe;

	for( int i=0; i<FRAMES; i++ ) {
		field.smooth_once( );

		std::ostringstream oss;
		oss << "/Users/dave/Desktop/inter" << i << ".mat";
		std::ofstream final_file{ oss.str() };
		fe = new MatlabFieldExporter( final_file);
		fe->exportField( field );
		delete fe;
	}
	
	std::ofstream final_file{ "/Users/dave/Desktop/final.mat" };
	fe = new MatlabFieldExporter( final_file);
	fe->exportField( field );
	delete fe;


    return 0;
}
