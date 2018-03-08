#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iostream>

#include <Graph/Graph.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

const int DIM = 50;
const float RADIUS = 10.0f;
const int SPHERE_THETA_STEPS = 20;
const int SPHERE_PHI_STEPS = 10;
const int CUBE_SIZE = 10;
const int TRI_RADIUS = 4;
const int SMOOTH_ITERATIONS = 400;
const int EXPORT_FRAMES = 400;
const std::string OUTPUT_DIRECTORY = "/Users/dave/Desktop/animesh_output";
const std::string OUTPUT_FILE_ROOT = "frame";

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


void write_matlab_file( Field * field, const std::string& file_name ) {
	std::ostringstream oss;
	oss << OUTPUT_DIRECTORY << "/" << file_name;
	std::ofstream file{ oss.str() };
	FieldExporter * fe = new MatlabFieldExporter( file );
	fe->exportField( *field );
	delete fe;
}

void write_matlab_file( Field * field, int index ) {
	std::ostringstream oss;
	oss << OUTPUT_FILE_ROOT << index << ".mat";
	write_matlab_file( field, oss.str());
}


int main( int argc, char * argv[] ) {

	Field * field = planar_field( );

	write_matlab_file( field, "initial.mat" );

	int index = 0;
	for( int i=0; i<SMOOTH_ITERATIONS; i++ ) {
		float cost = field->smooth_once( );
		std::cout << cost << std::endl;
		if( i % EXPORT_FRAMES == 0 )
			write_matlab_file( field, index++ );
	}
	
	write_matlab_file( field, "final.mat" );

	delete field;
    return 0;
}
