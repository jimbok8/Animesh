#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iostream>

#include <Graph/Graph.h>
#include <Graph/GridGraphBuilder.h>
#include <Graph/NearestNeighbourGraphBuilder.h>
#include <Element/Element.h>
#include <Field/Field.h>
#include <Field/FieldExporter.h>
#include <Field/MatlabFieldExporter.h>

using namespace Eigen;

const int DIM_X = 30;
const int DIM_Y = 30;
const float GRID_SPACING = 2.0f;
const float RADIUS = 10.0f;
const int SPHERE_THETA_STEPS = 30;
const int SPHERE_PHI_STEPS = 15;
const int CUBE_SIZE = 3;
const int TRI_RADIUS = 4;
const int SMOOTH_ITERATIONS = 100;
const int EXPORT_FRAMES = 10;
const std::string OUTPUT_DIRECTORY = "/Users/dave/Desktop/animesh_output";
const std::string OUTPUT_FILE_ROOT = "frame";



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
	bool make_field_fixed = true;
	bool dump_field = true;

//	Field * field = Field::spherical_field( RADIUS, SPHERE_THETA_STEPS, SPHERE_PHI_STEPS, make_field_fixed);
	Field * field = Field::planar_field( DIM_X, DIM_Y, GRID_SPACING, make_field_fixed);

	if(dump_field) {
		field->dump( );
	}

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
