/*
	Builder to construct a Field from a Point Cloud
	This will accept the PCL, iterate over all of the points in it to generate normals for each point and
	construct a vector of Elements which can then be used to build the Graph
 */

#include <Field/FieldBuilder.h>
#include <Eigen/Core>
#include <Graph/NearestNeighbourGraphBuilder.h>

/**
 * @param pcl The PointCloud
 * @return The Field
 */
 Field * build_field_from_point_cloud( const PointCloud * const point_cloud ) {
 	using namespace Eigen;

 	std::vector<Element> elements;

 	for( size_t p = 0; p < point_cloud->size(); ++p ) {
 		const Point& pt = point_cloud->point(p);
	 	Element e{ pt.location, pt.normal };
	 	elements.push_back( e );
 	}

	NearestNeighbourGraphBuilder * nngb = new NearestNeighbourGraphBuilder( 8 );
	Field * field = new Field( nngb, elements );

	delete nngb;

	return field;
 }