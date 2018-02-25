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
