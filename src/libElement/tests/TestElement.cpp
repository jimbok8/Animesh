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

    TEST( TestElement, EqualityOperatorShouldBeTrueForSameElements ) {
    	using namespace Eigen;

		Vector3f location{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	Element element1{ location, normal };

    	EXPECT_TRUE( element1 == element1 );
    }

    TEST( TestElement, EqualityOperatorShouldBeTrueForIdenticalElements ) {
    	using namespace Eigen;
    	
		Vector3f location{ 1.0f, 2.0f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	Element element1{ location, normal };
    	Element element2{ location, normal };

    	EXPECT_TRUE( element1 == element2 );
    }
    TEST( TestElement, EqualityOperatorShouldBeFalseIfLocationIsDifferent ) {
    	using namespace Eigen;
    	
		Vector3f location1{ 1.0f, 2.0f, 3.0f };
		Vector3f location2{ 1.0f, 2.1f, 3.0f };
		Vector3f normal{ 1.0f, 0.0f, 0.0f };

    	Element element1{ location1, normal };
    	Element element2{ location2, normal };

    	EXPECT_FALSE( element1 == element2 );
    }
    TEST( TestElement, EqualityOperatorShouldBeFalseIfNormalIsDifferent ) {
    	using namespace Eigen;
    	
		Vector3f location{ 1.0f, 2.0f, 3.0f };
		Vector3f normal1{ 1.0f, 0.0f, 0.0f };
		Vector3f normal2{ 0.0f, 0.0f, 1.0f };

    	Element element1{ location, normal1 };
    	Element element2{ location, normal2 };

    	EXPECT_FALSE( element1 == element2 );
    }
}