#include "gtest/gtest.h"
#include <Graph/Graph.h>
#include <Graph/GraphNode.h>

namespace {
    /* **********************************************************************
     * *                                                                    *
     * * GraphNode Constructor tests                                        *
     * *                                                                    *
     * **********************************************************************/
	TEST(TestGraph, ZeroLengthVectorsShouldThrow) { 
		using namespace Eigen; 
		
    	try {
    		Vector3f point{ 1.0f, 2.0f, 3.0f };
    		Vector3f normal{ 1.0f, 1.0f, 0.0f };

        	new GraphNode( point, normal );
        	FAIL() << "Expected std::invalid_argument";
    	}
	    catch ( std::invalid_argument const & err ){
	        EXPECT_EQ( err.what(), std::string( "Normal vector must be unit") );
	    }
	    catch( ... ) {
	        FAIL( ) <<"Expected std::invalid_argument";
	    }
    }


	TEST(TestGraph, TestNormalSetCorrectlyFromConstructor) { 
		using namespace Eigen; 
		
		Vector3f point{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	GraphNode graphNode{ point, normal };

    	EXPECT_TRUE( normal.isApprox(graphNode.normal() ) );
    }

	TEST(TestGraph, TestPointSetCorrectlyFromConstructor) { 
		using namespace Eigen; 
		
		Vector3f point{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	GraphNode graphNode{ point, normal };

    	EXPECT_TRUE( point.isApprox(graphNode.point() ) );
    }

    /* **********************************************************************
     * *                                                                    *
     * * Graph Constructor tests                                            *
     * *                                                                    *
     * **********************************************************************/
	TEST(TestGraph, TestPointsAndNormalsWithDifferentSizeShouldThrow) { 
		using namespace Eigen; 
		
		Matrix<float, 3, 4> points;
		Matrix<float, 3, 5> normals;
		MatrixXi neighbours;

    	try {
        	Graph{ points, normals, neighbours };
        	FAIL() << "Expected std::invalid_argument";
    	}
	    catch ( std::invalid_argument const & err ){
	        EXPECT_EQ( err.what(), std::string( "Points and normals must have the same dimensions") );
	    }
	    catch( ... ) {
	        FAIL( ) <<"Expected std::invalid_argument";
	    }
    }


	TEST(TestGraph, TestNeighboursIncorrectlySizedShouldThrow) { 
		using namespace Eigen; 
		
		Matrix<float, 3, 4> points;
		Matrix<float, 3, 4> normals;
		MatrixXi neighbours{5,5};

    	try {
        	Graph{ points, normals, neighbours };
        	FAIL() << "Expected std::invalid_argument";
    	}
	    catch ( std::invalid_argument const & err ){
	        EXPECT_EQ( err.what(), std::string( "Neighbours has incorrect dimensions") );
	    }
	    catch( ... ) {
	        FAIL( ) <<"Expected std::invalid_argument";
	    }
    }

    TEST(TestGraph, TestSizeIsCorrectlySet) { 
		using namespace Eigen; 
		
		Matrix<float, 3, 6> points;
		points << 1.0f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
		          0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  0.0f,
		          0.0f,  0.0f,  1.0f,  0.0f,  0.0f, -1.0f;
		Matrix<float, 3, 6> normals;
		normals << 1.0f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
		           0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  0.0f,
		           0.0f,  0.0f,  1.0f,  0.0f,  0.0f, -1.0f;

		MatrixXi neighbours{6,6};
		neighbours << 0, 1, 1, 0, 1, 1,
		              1, 0, 1, 1, 0, 1,
		              1, 1, 0, 1, 1, 0,
		              0, 1, 1, 0, 1, 1, 
		              1, 0, 1, 1, 0, 1,
		              1, 1, 0, 1, 1, 0;

    	Graph graph{ points, normals, neighbours };
        EXPECT_EQ( 6, graph.size() );
    }
}