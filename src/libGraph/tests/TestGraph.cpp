#include "gtest/gtest.h"
#include <Graph/GraphNode.h>

namespace {
    /* **********************************************************************
     * *                                                                    *
     * * Constructor tests                                                  *
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


}