#pragma once 

#include <unordered_map>

#include <Graph/Graph.h>
#include <Graph/GraphBuilder.h>
#include <Graph/GridGraphBuilder.h>
#include <Graph/NearestNeighbourGraphBuilder.h>

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
	 * Construct a planar field centred at (0,0,0)
	 * @param dim_x The number of points in the X plane
	 * @param dim_y The number of points in the Y plane
	 * @param grid_spacing The space between grid points
	 * @param make_fixed If true, set the field tangents to the lowest energy/solved position
	 */
	static Field * planar_field( std::size_t dim_x, std::size_t dim_y, float grid_spacing, bool make_fixed );

	/**
	 * Construct a spherical field centred at (0,0,0)
	 * @param radius The radius of the sphere to be constructed
	 * @param theta_steps The number of steps around the sphere (in XZ plane)
	 * @param phi_steps The number of steps in the Y direction
	 * @param make_fixed If true, set the field tangents to the lowest energy/solved position
	 */
	static Field * spherical_field( float radius, std::size_t theta_steps, std::size_t phi_steps, bool make_fixed);
	
	static Field * triangular_field( float tri_radius);
	static Field * cubic_field(std::size_t cube_size, bool make_fixed);

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

	/**
	 * Set all the tangents in the field to specific values.
	 * @param new_tangents The new tangents
	 * @return The difference between the old and new values
	 */
	float set_tangents( const std::vector<const Eigen::Vector3f>& new_tangents );

	void dump( ) const;

	void enable_tracing( bool enable_tracing ) { m_tracing_enabled = enable_tracing;}

private:
	/** The Graph - helps us get neghbours */
	Graph *  	m_graph;

	/** Flag to determine if we should trace field moothing */
	bool 		m_tracing_enabled;
};

/**
 * Given an arbitrary vector v, project it into the plane whose normal is given as n
 * also unitize it.
 * @param v The vector
 * @param n The normal
 * @return a unit vector in the tangent plane
 */
Eigen::Vector3f reproject_to_tangent_space( const Eigen::Vector3f& v, const Eigen::Vector3f& n);


/**
 * @param targetVector The vector we're trying to match
 * @param targetK The value of K for the target vector which should be locked
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_for( const Eigen::Vector3f& targetVector, 
									  const Eigen::Vector3f& targetNormal, 
									  int targetK, 
									  const Eigen::Vector3f& sourceVector, 
									  const Eigen::Vector3f& sourceNormal );

/**
 * @param targetVector The vector we're trying to match
 * @param normal The normal about which to rotate the sourceVector
 * @param sourceVector the vector to be matched
 * @return the best fitting vector (i.e. best multiple of PI/2 + angle)
 * 
 */
Eigen::Vector3f best_rosy_vector_by_dot_product( const Eigen::Vector3f& targetVector, 
									  const Eigen::Vector3f& targetNormal, 
									  int targetK, 
									  const Eigen::Vector3f& sourceVector, 
									  const Eigen::Vector3f& sourceNormal );
