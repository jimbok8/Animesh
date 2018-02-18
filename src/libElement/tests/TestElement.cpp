#include "gtest/gtest.h"
#include <Element/Element.h>

namespace {
    /* **********************************************************************
     * *                                                                    *
     * * Element Constructor tests                                          *
     * *                                                                    *
     * **********************************************************************/
	TEST(TestElement, ZeroLengthVectorsShouldThrow) { 
		using namespace Eigen; 
		
    	try {
    		Vector3f location{ 1.0f, 2.0f, 3.0f };
    		Vector3f normal{ 1.0f, 1.0f, 0.0f };

        	Element element( location, normal );
        	FAIL() << "Expected std::invalid_argument";
    	}
	    catch ( std::invalid_argument const & err ){
	        EXPECT_EQ( err.what(), std::string( "Normal vector must be unit") );
	    }
	    catch( ... ) {
	        FAIL( ) <<"Expected std::invalid_argument";
	    }
    }


	TEST(TestElement, NormalSetCorrectlyFromConstructor) { 
		using namespace Eigen; 
		
		Vector3f location{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	Element  element{ location, normal };

    	EXPECT_TRUE( normal.isApprox(element.normal() ) );
    }

	TEST(TestElement, LocationSetCorrectlyFromConstructor) { 
		using namespace Eigen; 
		
		Vector3f location{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	Element element{ location, normal };

    	EXPECT_TRUE( location.isApprox(element.location() ) );
    }

}