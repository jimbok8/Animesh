/*
	Builder to construct a Field from a Point Cloud
	This will accept the PCL, iterate over all of the points in it to generate normals for each point and
	construct a vector of Elements which can then be used to build the Graph
 */


#include <Field/Field.h>
#include <PointCloud/PointCloud.h>

/**
 * @param pcl The PointCloud
 * @return The Field
 */
 Field * build_field_from_point_cloud( const PointCloud * const point_cloud );