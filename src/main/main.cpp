#include <fstream>
#include <sstream>

#include <Graph/Graph.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

const int DIM = 50;
const int FRAMES = 10;
const float RADIUS = 10.0f;

Field * planar_field( ) {
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

	return new Field( g );
}

Field * spherical_field( ) {
	EdgeManager *em = new NNEdgeManager{ 8 };
	Graph *g = new Graph( em );

	float delta = ( 2 * M_PI ) / 40.0f;
	for( float theta = 0.0f; theta <= 2*M_PI; theta +=  delta) {
		for( float phi = 0.0f; phi<=M_PI/2.0f; phi += delta ) {
			Vector3f location { RADIUS * sin(phi) * cos(theta),  RADIUS * sin(phi) * sin(theta), RADIUS * cos( phi ) };
			Vector3f normal = location;
			normal.normalize();

			Element *e = new Element( location, normal );

			g->addElement( *e );
		}
	}

	return new Field( g );
}



int main( int argc, char * argv[] ) {

	Field * field = spherical_field( );


	std::ofstream initial_file{ "/Users/dave/Desktop/initial.mat" };
	FieldExporter * fe = new MatlabFieldExporter( initial_file );
	fe->exportField( *field );
	delete fe;

	for( int i=0; i<FRAMES; i++ ) {
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );
		field->smooth_once( );


		std::ostringstream oss;
		oss << "/Users/dave/Desktop/inter" << i << ".mat";
		std::ofstream final_file{ oss.str() };
		fe = new MatlabFieldExporter( final_file);
		fe->exportField( *field );
		delete fe;
	}
	
	std::ofstream final_file{ "/Users/dave/Desktop/final.mat" };
	fe = new MatlabFieldExporter( final_file);
	fe->exportField( *field );
	delete fe;
	delete field;


    return 0;
}
