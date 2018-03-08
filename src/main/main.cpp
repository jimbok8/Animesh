#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <Graph/Graph.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

const int DIM = 50;
const int FRAMES = 10;
const float RADIUS = 10.0f;
const int SPHERE_THETA_STEPS = 20;
const int SPHERE_PHI_STEPS = 10;
const int CUBE_SIZE = 10;
const int TRI_RADIUS = 4;

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
	EdgeManager *em = new NNEdgeManager{ 4 };
	Graph *g = new Graph( em );

	float maxTheta = 2 * M_PI;
	float maxPhi = M_PI / 2.0f;
	float deltaTheta = maxTheta / SPHERE_THETA_STEPS;
	float deltaPhi   = maxPhi / SPHERE_PHI_STEPS;
	for( float theta = 0.0f; theta < maxTheta; theta +=  deltaTheta) {
		for( float phi = 0.0f; phi<=maxPhi; phi += deltaPhi ) {
			Vector3f location { RADIUS * sin(phi) * cos(theta),  RADIUS * sin(phi) * sin(theta), RADIUS * cos( phi ) };
			Vector3f normal = location;
			normal.normalize();

			Element *e = new Element( location, normal );

			g->addElement( *e );
		}
	}

	return new Field( g );
}

Field * triangular_field( ) {
	EdgeManager *em = new NNEdgeManager{ 8 };
	Graph *g = new Graph( em );

	using namespace Eigen;

	Vector3f v1{ 0.0f, 0.0f, TRI_RADIUS };
	Vector3f v2{ TRI_RADIUS * -0.866f, 0.0f, TRI_RADIUS * -0.5f };
	Vector3f v3{ TRI_RADIUS *  0.866f, 0.0f, TRI_RADIUS * -0.5f };

	Vector3f vv1 = v3 - v2;
	Vector3f vv2 = v1 - v2;

	for( int i=0; i<200; i++ ) {

		float c1 = rand() %100 / 100.0f;
		float c2 = rand() %100 / 100.0f;

		if( c1 + c2 >= 1 ) {
			c1 = 1.0f - c1;
			c2 = 1.0f - c2;
		}

		Vector3f location  = v1 + c1 * vv1 + c2 * vv2;
		Vector3f normal{ 0, 1, 0 };
		Element *e = new Element( location, normal );
		g->addElement( *e );
	}

	return new Field( g );
}

Field * cubic_field( ) {
	EdgeManager *em = new NNEdgeManager{ 8 };
	Graph *g = new Graph( em );

	float minVal = 0.5f - ( CUBE_SIZE / 2.0f );
	float maxVal = ( CUBE_SIZE / 2.0f ) - 0.5f;


	Vector3f normalZ{ 0, 0, 1 };
	for( float x = minVal; x <= maxVal; x +=  1.0f) {
		for( float y = minVal; y <= maxVal; y+= 1.0f ) {

			Vector3f location { x, y, minVal - 0.5f };
			Element *e = new Element( location, -normalZ );
			g->addElement( *e );

			Vector3f location2 { x, y, maxVal + 0.5f };
			e = new Element( location2, normalZ );
			g->addElement( *e );
		}
	}
	Vector3f normalY{ 0, 1, 0 };
	for( float x = minVal; x <= maxVal; x +=  1.0f) {
		for( float z = minVal; z <= maxVal; z+= 1.0f ) {
			Vector3f location { x, minVal - 0.5f, z };
			Element *e = new Element( location, -normalY );
			g->addElement( *e );

			Vector3f location2 { x, maxVal + 0.5f, z };
			e = new Element( location2, normalY );
			g->addElement( *e );
		}
	}
	Vector3f normalX{ 1, 0, 0 };
	for( float z = minVal; z <= maxVal; z +=  1.0f) {
		for( float y = minVal; y <= maxVal; y+= 1.0f ) {
			Vector3f location { minVal - 0.5f, y, z };
			Element *e = new Element( location, -normalX );
			g->addElement( *e );

			Vector3f location2 { maxVal + 0.5f, y, z };
			e = new Element( location2, normalX );
			g->addElement( *e );
		}
	}

	return new Field( g );
}


int main( int argc, char * argv[] ) {

	Field * field = planar_field( );


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
