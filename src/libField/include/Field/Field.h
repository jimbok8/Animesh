#pragma once 

#include <unordered_map>

#include "FieldData.h"
#include <Graph/Graph.h>
#include <Graph/GraphBuilder.h>

#include <unordered_map>

class Field {
public:
	/**
	 * Construct the field using a graph builder and some elements
	 */
	Field( const GraphBuilder * const graph_builder, const std::vector<Element>& elements );

	~Field( );

	/**
	 * @return the size of the ifled
	 */
	std::size_t size() const;


	/**
	 * Smooth the field once, applying smoothing to each node
	 * @return the largest error in tangent
	 */
	float smooth_once( );

	/**
	 * Smooth node
	 * Smooth a node in the field by averaging it's neighbours
	 * @return The new vector.
	 */
	Eigen::Vector3f get_smoothed_tangent_data_for_node( const GraphNode * const gn ) const;


	/**
	 * Return vector of elements */
	const std::vector<const FieldElement *> elements( ) const;

private:
	/** The Graph - helps us get neghbours */
	Graph *  	m_graph;
};

/**
 * Given an arbitrary vector v, project it into the plane whose normal is given as n
 * also unitize it.
 * @param v The vector
 * @param n The normal
 * @return a unit vector in the tangent plane
 */
Eigen::Vector3f reproject_to_tangent_space( const Eigen::Vector3f& v, const Eigen::Vector3f& n);
