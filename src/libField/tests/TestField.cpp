#include "TestField.h"

#include <Graph/Graph.h>
#include <Field/Field.h>


void TestField::SetUp( ) {}
void TestField::TearDown( ) {}

TEST_F(TestField, SizeAccuatelyReportsSize) { 
    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

    EXPECT_EQ( 3, field.size() );
}


TEST_F(TestField, ShouldThrowWhenRequestingOutOfRangeItem) { 
    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

 	try {
        field.dataForGraphNode( 4 );
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Index out of range") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}

TEST_F(TestField, GetByIndexReturnsCorrectLocation) { 
    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> tuple = field.dataForGraphNode( 1 );

    EXPECT_EQ( el_1_1_2.location(), std::get<0>(tuple) );
}

TEST_F(TestField, GetByIndexReturnsCorrectNormal) { 
    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> tuple = field.dataForGraphNode( 1 );

    EXPECT_EQ( el_1_1_2.normal(), std::get<1>(tuple) );
}

TEST_F(TestField, GetByIndexReturnsCorrectTangent) { 
    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

    std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f> tuple = field.dataForGraphNode( 1 );

    Eigen::Vector3f tangent = std::get<2>(tuple);

    EXPECT_FLOAT_EQ( 0.0f, el_1_1_2.normal().dot( tangent ) );
}


//
// Test average vector
//
TEST_F( TestField, AverageOfNoVectorsShouldThrow ) {
    using namespace Eigen;
    const std::vector<Eigen::Vector3f> source;

    try {
        Vector3f mean = compute_mean_vector( source );
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Vector can't be empty") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}


TEST_F( TestField, AverageOfOneVectorShouldBeTheVector ) {
    using namespace Eigen;

    std::vector<Vector3f> source;

    Vector3f v1{ 1.0f, 2.0f, 3.0f };
    source.push_back( v1 );

    Vector3f mean = compute_mean_vector( source );

    EXPECT_FLOAT_EQ( 1.0f, mean[0] );
    EXPECT_FLOAT_EQ( 2.0f, mean[1] );
    EXPECT_FLOAT_EQ( 3.0f, mean[2] );
}


TEST_F( TestField, AverageOfSameVectorShouldBeInputVector ) {
    using namespace Eigen;

    std::vector<Vector3f> source;

    Vector3f v1{ 1.0f, 2.0f, 3.0f };
    source.push_back( v1 );

    Vector3f v2{ 1.0f, 2.0f, 3.0f };
    source.push_back( v2 );

    Vector3f mean = compute_mean_vector( source );

    EXPECT_FLOAT_EQ( 1.0f, mean[0] );
    EXPECT_FLOAT_EQ( 2.0f, mean[1] );
    EXPECT_FLOAT_EQ( 3.0f, mean[2] );
}


TEST_F( TestField, AverageOfDifferentVectorsShouldBecorrect ) {
    using namespace Eigen;

    std::vector<Vector3f> source;

    Vector3f v1{ 1.0f, 2.0f, 3.0f };
    source.push_back( v1 );

    Vector3f v2{ 3.0f, 2.0f, 1.0f };
    source.push_back( v2 );

    Vector3f mean = compute_mean_vector( source );

    EXPECT_FLOAT_EQ( 2.0f, mean[0] );
    EXPECT_FLOAT_EQ( 2.0f, mean[1] );
    EXPECT_FLOAT_EQ( 2.0f, mean[2] );
}

TEST_F( TestField, SizeOfaveragesVectorIsSizeOfNeghbours ) {
    using namespace Eigen; 

    Graph graph{ new NNEdgeManager{ 3} };
    graph.addElement( el_1_1_1 );
    graph.addElement( el_1_1_2 );
    graph.addElement( el_1_1_3 );

    Field field{ &graph };

    const GraphNode * gn = graph.node( 1 );
    std::vector<Vector3f> best_vectors = field.best_rosy_vectors_for_neighbours_of_node( gn );

    EXPECT_EQ( 2, best_vectors.size() );
}
