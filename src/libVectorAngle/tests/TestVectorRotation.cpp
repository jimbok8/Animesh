#include "gtest/gtest.h"
#include <VectorAngle/angleBetweenVectors.h>

/*
* Test that rotating a vector around another unit vector gives the correct outptu
*/

namespace {

    TEST(TestVectorRotation, ZeroLengthOShouldThrow) { 
        Eigen::Vector3f o{ 0.0f, 0.0f, 0.0f };
        Eigen::Vector3f n{ 1.0f, 0.0f, 0.0f };
        int k = 0;
        
        try {
            vectorByRotatingOAroundN(o, n, k);
            FAIL() << "Expected std::invalid_argument";
       }
        catch ( std::invalid_argument const & err ){
            EXPECT_EQ( err.what(), std::string( "Vector may not be zero length") );
        }
        catch( ... ) {
            FAIL( ) <<"Expected std::invalid_argument";
        }
    }

    TEST(TestVectorRotation, NonUnitNormalShouldThrow) { 
        Eigen::Vector3f o{ 1.0f, 0.0f, 0.0f };
        Eigen::Vector3f n{ 1.0f, 1.0f, 0.0f };
        int k = 0;
        
        try {
            vectorByRotatingOAroundN(o, n, k);
            FAIL() << "Expected std::invalid_argument";
       }
        catch ( std::invalid_argument const & err ){
            EXPECT_EQ( err.what(), std::string( "Normal must be unit vector") );
        }
        catch( ... ) {
            FAIL( ) <<"Expected std::invalid_argument";
        }
    }

    /* ********************************************************************************
     * ** Test rotations in the XY plane
     * ********************************************************************************/

    TEST( TestVectorRotation, Rotate_2_1_0_about_0_0_1_by_0 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 1.0f, 0.0f };
        Vector3f n{ 0.0f, 0.0f, 1.0f };
        int k = 0;

        Vector3f expected{ 2.0f, 1.0f, 0.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_1_0_about_0_0_1_by_1 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 1.0f, 0.0f };
        Vector3f n{ 0.0f, 0.0f, 1.0f };
        int k = 1;

        Vector3f expected{ -1.0f, 2.0f, 0.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_1_0_about_0_0_1_by_2 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 1.0f, 0.0f };
        Vector3f n{ 0.0f, 0.0f, 1.0f };
        int k = 2;

        Vector3f expected{ -2.0f, -1.0f, 0.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_1_0_about_0_0_1_by_3 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 1.0f, 0.0f };
        Vector3f n{ 0.0f, 0.0f, 1.0f };
        int k = 3;

        Vector3f expected{ 1.0f, -2.0f, 0.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    /* ********************************************************************************
     * ** Test rotations in the XZ plane
     * ********************************************************************************/

    TEST( TestVectorRotation, Rotate_2_0_1_about_0_1_0_by_0 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 0.0f, 1.0f };
        Vector3f n{ 0.0f, 1.0f, 0.0f };
        int k = 0;

        Vector3f expected{ 2.0f, 0.0f, 1.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_0_1_about_0_1_0_by_1 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 0.0f, 1.0f };
        Vector3f n{ 0.0f, 1.0f, 0.0f };
        int k = 1;

        Vector3f expected{ 1.0f, 0.0f, -2.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_0_1_about_0_1_0_by_2 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 0.0f, 1.0f };
        Vector3f n{ 0.0f, 1.0f, 0.0f };
        int k = 2;

        Vector3f expected{ -2.0f, 0.0f, -1.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_2_0_1_about_0_1_0_by_3 ) {
        using namespace Eigen;

        Vector3f o{ 2.0f, 0.0f, 1.0f };
        Vector3f n{ 0.0f, 1.0f, 0.0f };
        int k = 3;

        Vector3f expected{ -1.0f, 0.0f, 2.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    /* ********************************************************************************
     * ** Test rotations in the YZ plane
     * ********************************************************************************/

    TEST( TestVectorRotation, Rotate_0_1_2_about_1_0_0_by_0 ) {
        using namespace Eigen;

        Vector3f o{ 0.0f, 1.0f, 2.0f };
        Vector3f n{ 1.0f, 0.0f, 0.0f };
        int k = 0;

        Vector3f expected{ 0.0f, 1.0f, 2.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_0_1_2_about_1_0_0_by_1 ) {
        using namespace Eigen;

        Vector3f o{ 0.0f, 1.0f, 2.0f };
        Vector3f n{ 1.0f, 0.0f, 0.0f };
        int k = 1;

        Vector3f expected{ 0.0f, -2.0f, 1.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_0_1_2_about_1_0_0_by_2 ) {
        using namespace Eigen;

        Vector3f o{ 0.0f, 1.0f, 2.0f };
        Vector3f n{ 1.0f, 0.0f, 0.0f };
        int k = 2;

        Vector3f expected{ 0.0f, -1.0f, -2.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    TEST( TestVectorRotation, Rotate_0_1_2_about_1_0_0_by_3 ) {
        using namespace Eigen;

        Vector3f o{ 0.0f, 1.0f, 2.0f };
        Vector3f n{ 1.0f, 0.0f, 0.0f };
        int k = 3;

        Vector3f expected{ 0.0f, 2.0f, -1.0f};
        Vector3f actual = vectorByRotatingOAroundN( o, n, k );

        EXPECT_FLOAT_EQ( expected[0], actual[0] );
        EXPECT_FLOAT_EQ( expected[1], actual[1] );
        EXPECT_FLOAT_EQ( expected[2], actual[2] );
    }

    
}
