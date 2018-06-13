#include <Field/Field.h>
#include <Element/Element.h>
#include <RoSy/RoSy.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <Graph/GridGraphBuilder.h>
#include <Graph/NearestNeighbourGraphBuilder.h>
#include <Graph/ExplicitGraphBuilder.h>
#include <pcl/kdtree/kdtree_flann.h>

//#define TRACE  1

#include <iostream>
#include <queue>
#include <set>

Field::~Field( ) {
	delete m_graph;
}


void Field::init( const GraphBuilder<void*> * const graph_builder, const std::vector<Element>& elements ) {
	m_graph = graph_builder->build_graph_for_elements( elements );

	// Initialise field tangents to random values
	for( auto node_iter = m_graph->m_data_to_node_map.begin(); node_iter != m_graph->m_data_to_node_map.end(); ++node_iter ) {
		FieldElement * fe = (FieldElement *) (*node_iter).first;


		Eigen::Vector3f random = Eigen::VectorXf::Random(3);
		random = random.cross( fe->m_normal ).normalized( );
		fe->m_tangent = random;
	}
}


/**
 * Construct the field using a graph builder and some elements
 */
Field::Field( const GraphBuilder<void*> * const graph_builder, const std::vector<Element>& elements ) {
	init( graph_builder, elements);
}

FieldElement * field_element_from_point( const pcl::PointNormal& point ) {
	FieldElement * fe = new FieldElement( 
		Eigen::Vector3f{ point.x * 100, point.y * 100, point.z * 100},
		Eigen::Vector3f{ point.normal_x, point.normal_y, point.normal_z},
		Eigen::Vector3f::Zero());
	return fe;
}

bool operator == (const pcl::PointNormal& point1, const pcl::PointNormal& point2 ) {
	return (point1.x == point2.x) &&
		(point1.y == point2.y) &&
		(point1.z == point2.z) &&
		(point1.normal_x == point2.normal_x) &&
		(point1.normal_y == point2.normal_y) &&
		(point1.normal_z == point2.normal_z);
}

/*
 * Comparator for pcl::PointNormal instances
 */
struct cmpPointNormal {
	bool operator()(const pcl::PointNormal& a, const pcl::PointNormal& b) const {

		float v1x = (a.x + a.normal_x);
		float v1y = (a.y + a.normal_y);
		float v1z = (a.z + a.normal_z);

		float v2x = (b.x + b.normal_x);
		float v2y = (b.y + b.normal_y);
		float v2z = (b.z + b.normal_z);

		return (v1x*v1x+v1y*v1y+v1z*v1z) < (v2x*v2x+v2y*v2y+v2z*v2z);
    }
};

FieldElement * find_element_or_throw( const std::map<pcl::PointNormal, FieldElement*, cmpPointNormal>& map, const pcl::PointNormal& point ) {
	auto find_it = map.find(point);
	if( find_it == map.end() )
		throw std::runtime_error( "No FE found for point" );
	return find_it->second;
}


/**
 * Construct a field given a point cloud
 */
Field::Field( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int k ) {

	// First make an empty Graph
    m_graph = new Graph<FieldElement*, void*>();


	// Next add all the points to it
	std::map<pcl::PointNormal, FieldElement*, cmpPointNormal> point_to_fe_map;
	size_t points_added = 0;
    for( auto it = cloud->begin(); it != cloud->end(); ++it ) {
    	pcl::PointNormal point = *it;
		FieldElement * fe = field_element_from_point( point );
		m_graph->add_node( fe );

		// Keep a mapping from point to FE
		point_to_fe_map[ point ] = fe;

		++points_added;
	}
	if( m_tracing_enabled )
		std::cout<< "Added " << points_added << " points" << std::endl;



    // Set up for kNN
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud (cloud);
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);


	// Keep track of points visited
	std::set<pcl::PointNormal, cmpPointNormal> visited;

	// For each point, construct edges
    for( auto it = cloud->begin(); it != cloud->end(); ++it ) {
    	pcl::PointNormal this_point = *it;
		if( m_tracing_enabled )
			std::cout<< "Processing point " << this_point << std::endl;

		// Look up FE
		FieldElement * this_fe = find_element_or_throw( point_to_fe_map, this_point);

    	// Now find neighbours
    	pointIdxNKNSearch.clear();
    	pointNKNSquaredDistance.clear();
        if ( kdtree.nearestKSearch (this_point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {

        	// ... and add edges to them
			if( m_tracing_enabled )
				std::cout<< "  Looking at neighbours" << std::endl;

            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
            	pcl::PointNormal neighbour_point = cloud->points[ pointIdxNKNSearch[i] ];

            	// Provided the eighbour isn't this point
            	if( ! (neighbour_point == this_point ) ){
					FieldElement * neighbour_fe = find_element_or_throw( point_to_fe_map, neighbour_point);

            	    m_graph->add_edge( this_fe, neighbour_fe, 1.0f, nullptr );
					if( m_tracing_enabled )
						std::cout<< "    Add neighbour " << neighbour_point << std::endl;
            	}
            	else {
					if( m_tracing_enabled )
						std::cout<< "    Ignoring me as my own neighbour" << std::endl;
            	}
        	}
        }
	}

    // Graph is built
    std::cout << "Done" << std::endl;

	// Initialise field tangents to random values
	for( auto node_iter = m_graph->m_data_to_node_map.begin(); node_iter != m_graph->m_data_to_node_map.end(); ++node_iter ) {
		FieldElement * fe = (FieldElement *) (*node_iter).first;

		Eigen::Vector3f random = Eigen::VectorXf::Random(3);
		random = random.cross( fe->m_normal ).normalized( );
		fe->m_tangent = random;
	}
}

/**
 * Construct a planar field centred at (0,0,0)
 * @param dim_x The number of points in the X plane
 * @param dim_y The number of points in the Y plane
 * @param grid_spacing The space between grid points
 * @param make_fixed If true, set the field tangents to the lowest energy/solved position
 */
Field * Field::planar_field( std::size_t dim_x, std::size_t dim_y, float grid_spacing, bool make_fixed ) {
	using namespace Eigen;

	std::vector<Element> elements;
	for( int y = 0; y<dim_y; y++ ) {
		for( int x = 0; x<dim_x; x++ ) {
			Vector3f location { x * grid_spacing, y * grid_spacing, 0.0f };
			Vector3f normal{ 0.0f, 0.0f, 1.0f };
			Element e{location, normal };
			elements.push_back( e );
		}
	}

	GridGraphBuilder<void*> * ggb = new GridGraphBuilder<void*>( grid_spacing );
	Field * field = new Field( ggb, elements );
	delete ggb;

	// Make a set of planar tangents
	if( make_fixed ) {
		std::vector< const Eigen::Vector3f > good_tangents;
		for( std::size_t i=0; i< field->size(); ++i ) {
			float noise = random( );
			good_tangents.push_back( Eigen::Vector3f{ 0.0f, 1.0f, 0.0f } );
		}

		field->set_tangents( good_tangents );
	}

	return field;
}

/**
 * Construct a curved field centred at (0,0,0). THe field is defined by z=1-\frac{1}{10}\left( x^{2}+y^{2} \right)
 * with grid neighbourhod
 * @param dim_x The number of points in the X plane
 * @param dim_y The number of points in the Y plane
 * @param grid_spacing The space between grid points
 * @param make_fixed If true, set the field tangents to the lowest energy/solved position
 */
Field * Field::polynomial_field( std::size_t dim_x, std::size_t dim_y, float grid_spacing, bool make_fixed ) {
	using namespace Eigen;

	std::vector<Element> elements;
	int minx = -dim_x/2, maxx = minx+dim_x-1;
	int miny = -dim_y/2, maxy = miny+dim_y-1;
	for( int y = miny; y <= maxy; y++ ) {
		for( int x = minx; x <= maxx; x++ ) {
			float xc = x * grid_spacing;
			float yc = y * grid_spacing;
			float divisor = (10.0f * grid_spacing * grid_spacing);
			float zc = 1 - (xc*xc+yc*yc)/divisor;

			Vector3f location { xc, yc, zc };

			// Normals is (dz/dx, dz/dy, -1)
			Vector3f normal{ (2 * xc) / divisor, (2 * yc) / divisor, 1.0f };
			normal.normalize();
			Element e{location, normal };
			elements.push_back( e );
		}
	}

	// Explicitly set neighbours
	std::map<int,std::vector<int>> adjacency_map{};
	int idx = 0;
	for( int y = miny; y <= maxy; y++ ) {
		for( int x = minx; x <= maxx; x++ ) {
			std::vector<int> neighbours;
			if( x > minx ) neighbours.push_back( y * dim_x + x - 1 );
			if( x < maxx ) neighbours.push_back( y * dim_x + x + 1 );
			if( y > miny ) neighbours.push_back( y * dim_x - dim_x + x );
			if( y < maxy ) neighbours.push_back( y * dim_x + dim_x + x );
			adjacency_map[idx] = neighbours;
		}
	}

	ExplicitGraphBuilder<void*> * egb = new ExplicitGraphBuilder<void*>( adjacency_map );
	Field * field = new Field( egb, elements );
	delete egb;

	return field;
}




/**
 * Construct a spherical field centred at (0,0,0)
 * @param radius The radius of the sphere to be constructed
 * @param theta_steps The number of steps around the sphere (in XZ plane)
 * @param phi_steps The number of steps in the Y direction
 * @param make_fixed If true, set the field tangents to the lowest energy/solved position
 */
Field * Field::spherical_field( float radius, std::size_t theta_steps, std::size_t phi_steps, int k, bool make_fixed ) {
	using namespace Eigen;

	float maxTheta = M_PI;
	float maxPhi = M_PI / 2.0f;
	float deltaTheta = maxTheta / theta_steps;
	float deltaPhi   = maxPhi / phi_steps;

	std::vector<Element> elements;
	for( float theta = 0.0f; theta < maxTheta; theta +=  deltaTheta) {
		for( float phi = maxPhi; phi>=0; phi -= deltaPhi ) {

			Vector3f location { radius * sin(phi) * cos(theta),   radius * cos( phi ), radius * sin(phi) * sin(theta) };
			// Force random location on sphere
			// location = Vector3f::Random();
			// location = location.normalized() * radius;

			Vector3f normal = location.normalized();

			Element e{ location, normal };
			elements.push_back( e );
		}
	}
	Vector3f location { radius * sin(maxPhi) * cos(maxTheta),   radius * cos( maxPhi ), radius * sin(maxPhi) * sin(maxTheta) };
	Vector3f normal = location.normalized();
	Element e{ location, normal };
	elements.push_back( e );

	NearestNeighbourGraphBuilder<void*> * gb = new NearestNeighbourGraphBuilder<void*>( k );
	Field * field = new Field( gb, elements );
	delete gb;

	if( make_fixed ) {
		for( auto pair_iter = field->m_graph->m_data_to_node_map.begin();
			      pair_iter != field->m_graph->m_data_to_node_map.end();
			      ++pair_iter ) {

			FieldElement *fe = (FieldElement *)((*pair_iter).first);

			// Find cross product of normal and vertical
			Vector3f v{0.0f, 1.0f, 0.0f };
			if( std::fabs( fe->m_normal[1] - 1.0f ) < EPSILON ) {
				v = Vector3f{1.0f, 0.0f, 0.0f };
			}
			Vector3f hor_tan = v.cross( fe->m_normal );
			Vector3f ver_tan = hor_tan.cross(fe->m_normal);
			fe->m_tangent    = ver_tan.normalized( );
		}
	}

	return field;
}

Field * Field::triangular_field( float tri_radius ) {
	using namespace Eigen;

	Vector3f v1{ 0.0f, 0.0f, tri_radius };
	Vector3f v2{ tri_radius * -0.866f, 0.0f, tri_radius * -0.5f };
	Vector3f v3{ tri_radius *  0.866f, 0.0f, tri_radius * -0.5f };

	Vector3f vv1 = v3 - v2;
	Vector3f vv2 = v1 - v2;

	std::vector<Element> elements;
	for( int i=0; i<200; i++ ) {

		float c1 = rand() %100 / 100.0f;
		float c2 = rand() %100 / 100.0f;

		if( c1 + c2 >= 1 ) {
			c1 = 1.0f - c1;
			c2 = 1.0f - c2;
		}

		Vector3f location  = v1 + c1 * vv1 + c2 * vv2;
		Vector3f normal{ 0, 1, 0 };
		Element e{ location, normal };
		elements.push_back( e );
	}
	NearestNeighbourGraphBuilder<void*> * gb = new NearestNeighbourGraphBuilder<void*>( 8 );
	Field * field = new Field( gb, elements );

	delete gb;

	return field;
}

Field * Field::circular_field( float radius, int k, bool make_fixed ) {
	using namespace Eigen;
	std::vector<Element> elements;

	for ( int i=0; i<2; ++i) {
		float theta = 0.0f;
		for( int i = 0; i<40; i++ ) {
			float x = radius * std::cos( theta );
			float y = radius * std::sin( theta );

			Vector3f location{ x, y, 0.0f};
			Vector3f normal{ 0, 0, 1 };
			Element e{ location, normal };
			elements.push_back( e );

			theta += (2 * M_PI / 40 );
		}
		radius -= 1.0f;
	}

	NearestNeighbourGraphBuilder<void*> * gb = new NearestNeighbourGraphBuilder<void*>( k );
	Field * field = new Field( gb, elements );
	delete gb;

	// Make a set of planar tangents
	if( make_fixed ) {
		std::vector< const Eigen::Vector3f > good_tangents;
		for( std::size_t i=0; i< field->size(); ++i ) {
			good_tangents.push_back( Eigen::Vector3f{ 0.0f, 1.0f, 0.0f } );
		}

		field->set_tangents( good_tangents );
	}

	return field;
}

Field * Field::cubic_field( std::size_t cube_size, float scale, bool make_fixed) {
	using namespace Eigen;

	float minVal = (0.5f - ( cube_size / 2.0f )) * scale;
	float maxVal = (( cube_size / 2.0f ) - 0.5f) * scale;

	std::vector<Element> elements;

	Vector3f normalZ{ 0, 0, 1 };
	for( float x = minVal; x <= maxVal; x +=  scale) {
		for( float y = minVal; y <= maxVal; y+= scale ) {

			Vector3f location { x, y, minVal - (scale / 2.0f) };
			Element e{ location, -normalZ };
			elements.push_back( e );

			Vector3f location2 { x, y, maxVal + (scale / 2.0f) };
			Element e2{ location2, normalZ };
			elements.push_back( e2 );
		}
	}
	Vector3f normalY{ 0, 1, 0 };
	for( float x = minVal; x <= maxVal; x +=  scale) {
		for( float z = minVal; z <= maxVal; z+= scale ) {
			Vector3f location { x, minVal - (scale / 2.0f), z };
			Element e{ location, -normalY };
			elements.push_back( e );

			Vector3f location2 { x, maxVal + (scale / 2.0f), z };
			Element e2 {location2, normalY };
			elements.push_back( e2 );
		}
	}
	Vector3f normalX{ 1, 0, 0 };
	for( float z = minVal; z <= maxVal; z +=  scale) {
		for( float y = minVal; y <= maxVal; y+= scale ) {
			Vector3f location { minVal - (scale / 2.0f), y, z };
			Element e{ location, -normalX };
			elements.push_back( e );

			Vector3f location2 { maxVal + (scale / 2.0f), y, z };
			Element e2{location2, normalX };
			elements.push_back( e2 );
		}
	}

	NearestNeighbourGraphBuilder<void*> * gb = new NearestNeighbourGraphBuilder<void*>( 8 );
	Field * field = new Field( gb, elements );
	delete gb;


	if( make_fixed ) {
		for( auto pair_iter = field->m_graph->m_data_to_node_map.begin();
			      pair_iter != field->m_graph->m_data_to_node_map.end();
			      ++pair_iter ) {

			FieldElement *fe = (FieldElement *)((*pair_iter).first);

			if( fabsf(fe->m_tangent[0]) < EPSILON && 
				fabsf(fe->m_tangent[1]) < EPSILON && 
				fabsf(fe->m_tangent[2]) < EPSILON ) {
				throw std::invalid_argument( "Found zero tangent for fe");
			}

			if( fe->m_normal[0] < -EPSILON || fe->m_normal[0] > EPSILON || fe->m_normal[2] < -EPSILON || fe->m_normal[2] > EPSILON ) {
				fe->m_tangent = Vector3f{ 0.0f, 1.0f, 0.0f };
			} else if ( fe->m_normal[1] < -EPSILON || fe->m_normal[1] > EPSILON ) {
				fe->m_tangent = Vector3f{ 0.0f, 0.0f, 1.0f };
			} else {
				throw std::invalid_argument( "Unexpected normal in cubic_field");
			}
		}
	}

	return field;
}

/**
 * @return the size of the ifled
 */
std::size_t Field::size() const {
	return m_graph->m_data_to_node_map.size();
}

/**
 * @return the smoothness of the entire Field
 */
float Field::error( ) const {
	// E(O, k) :=      (oi, Rso (oji, ni, kij ))
	// For each node
	float error = 0.0f;
	for( auto map_iter = m_graph->m_data_to_node_map.begin(); map_iter != m_graph->m_data_to_node_map.end(); ++map_iter ) {
		GraphNode<FieldElement *, void*> * g = (*map_iter).second;
		error += get_error_for_node( g );
	}
	return error;
}

/**
 * @return the smoothness of one node
 */
float Field::get_error_for_node( const GraphNode<FieldElement *, void*>* gn ) const {
	float error = 0.0f;
	FieldElement * this_fe = (FieldElement *) gn->m_data;
	for( auto edge_iter = gn->m_edges.begin(); edge_iter != gn->m_edges.end(); ++edge_iter ) {
		const GraphNode<FieldElement *, void*> * neighbouring_node = std::get<2>(*edge_iter);
		FieldElement * neighbour_fe = (FieldElement *) neighbouring_node->m_data;

		std::pair<Eigen::Vector3f, Eigen::Vector3f> result = best_rosy_vector_pair( 
			this_fe->m_tangent,
			this_fe->m_normal,
			neighbour_fe->m_tangent, 
			neighbour_fe->m_normal);

		float theta = angle_between_vectors( result.first, result.second );
		error += (theta*theta);
	}
	return error;
}


/**
 * Smooth the field once, applying smoothing to each node
 * @return the largest error in tangent
 */
void Field::smooth_once( ) {
	using namespace Eigen;
	using namespace std;

	if( m_tracing_enabled )
		cout << "smooth_once" << endl;

	vector<const Vector3f> new_tangents;

	// For each graphnode, compute the smoothed tangent
	for( auto map_iter = m_graph->m_data_to_node_map.begin(); map_iter != m_graph->m_data_to_node_map.end(); ++map_iter ) {
		GraphNode<FieldElement *, void*> * g = (*map_iter).second;
		Vector3f new_tangent = smooth_node( g );
		new_tangents.push_back( new_tangent );
	}

	// Now update all of the nodes
	size_t vi = 0;
	for(auto map_iter = m_graph->m_data_to_node_map.begin();  map_iter != m_graph->m_data_to_node_map.end(); ++map_iter, ++vi ) {
		GraphNode<FieldElement *, void*> * g = (*map_iter).second;

		((FieldElement *) g->m_data)->m_tangent = new_tangents[vi];
	}
}


/**
 * Smooth the specified node (and neighbours)
 * @return The new vector.
 */
Eigen::Vector3f Field::smooth_node( const GraphNode<FieldElement *, void*> * const gn ) const {
	using namespace Eigen;

	FieldElement * this_fe = (FieldElement *) gn->m_data;
	if( m_tracing_enabled ) trace_node( "get_smoothed_tangent_data_for_node", this_fe);

	Vector3f sum = this_fe->m_tangent;
	float weight_sum = 0.0f;

	// For each edge from this node
	for( auto edge_iter = gn->m_edges.begin(); edge_iter != gn->m_edges.end(); ++edge_iter ) {

		// Get the adjacent FieldElement
		const GraphNode<FieldElement *, void*> * neighbouring_node = std::get<2>(*edge_iter);
		FieldElement * neighbour_fe = (FieldElement *) neighbouring_node->m_data;
		if( m_tracing_enabled ) trace_node( "    consider neighbour", neighbour_fe );

		// Find best matching rotation
		std::pair<Vector3f, Vector3f> result = best_rosy_vector_pair( 
			sum,
			this_fe->m_normal,
			neighbour_fe->m_tangent, 
			neighbour_fe->m_normal);

		// Update the computed new tangent
		float weight = std::get<0>(*edge_iter);
		sum = result.first * weight_sum + result.second * weight;
		weight_sum += weight;
		sum = reproject_to_tangent_space( result.second, this_fe->m_normal );
		sum.normalize();
	}
	return sum;
}


/**
 * Set all the tangents in the field to specific values.
 * @param new_tangents The new tangents
 * @return The difference between the old and new values
 */
void Field::set_tangents( const std::vector<const Eigen::Vector3f>& new_tangents ) {
	using namespace Eigen;

	if( new_tangents.size() != m_graph->m_data_to_node_map.size() ) 
		throw std::invalid_argument( "Must be one tangent for each node");

	auto tan_iter = new_tangents.begin();
	for( auto map_iter = m_graph->m_data_to_node_map.begin(); map_iter != m_graph->m_data_to_node_map.end(); ++map_iter, ++tan_iter ) {
		FieldElement * fe = (FieldElement *) (*map_iter).first;

		const Vector3f current_tangent = fe->m_tangent;
		const Vector3f new_tangent = (*tan_iter);
		fe->m_tangent = new_tangent; //(0.5 * fe->m_tangent + 0.5 * new_tangent).normalized();
	}
}

/**
 * Dump the field to stdout. For each point in the field dump the neighbour locations
 * and the field tangent
 */
void Field::dump(  ) const {
	for( auto pair_iter = m_graph->m_data_to_node_map.begin();
		      pair_iter != m_graph->m_data_to_node_map.end();
		      ++pair_iter ) {

		FieldElement *fe = (FieldElement *)((*pair_iter).first);
		GraphNode<FieldElement *, void*> *gn    = (GraphNode<FieldElement *, void*>*)    ((*pair_iter).second);

		std::cout << "locn    (" << fe->m_location[0] << "," << fe->m_location[1] << "," << fe->m_location[2] << ")" << std::endl;
		std::cout << "tangent (" << fe->m_tangent[0] << "," << fe->m_tangent[1] << "," << fe->m_tangent[2] << std::endl;
		std::cout << "neighbours " << std::endl;
		for( auto edge_iter  = gn->m_edges.begin();
			      edge_iter != gn->m_edges.end();
			      ++edge_iter ) {
			GraphNode<FieldElement *, void*> * gn = std::get<2>(*edge_iter);
			FieldElement * fe_n = (FieldElement *) (gn->m_data);

			std::cout << "      " << fe_n->m_location[0] << "," << fe_n->m_location[1] << "," << fe_n->m_location[2] << std::endl;
		}
	}
}

void Field::trace_node( const std::string& prefix, const FieldElement * this_fe ) const {
	std::cout << prefix 
		<< "( l=( " 
		<< this_fe->m_location[0] << ", "   
	 	<< this_fe->m_location[1] << ", "   
	 	<< this_fe->m_location[2] << "), t=(" 
		<< this_fe->m_tangent[0] << ", "   
	 	<< this_fe->m_tangent[1] << ", "   
	 	<< this_fe->m_tangent[2] << ")" 
	 	<< std::endl;
}

void Field::trace_vector( const std::string& prefix, const Eigen::Vector3f& vector ) const {
	std::cout << prefix  
		<< vector[0] << ", "   
	 	<< vector[1] << ", "   
	 	<< vector[2] << ") " 
	 	<< std::endl;
}
