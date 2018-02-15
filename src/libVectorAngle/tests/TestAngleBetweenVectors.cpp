//#include "TestAngleBetweenVectors.h"
#include "gtest/gtest.h"
#include <VectorAngle/angleBetweenVectors.h>

namespace {
TEST(TestAngleBetweenVectors, ZeroLengthVectorsShouldThrow) { 
    Eigen::Vector3f v1{ 0.1f, 0.2f, 0.3f };
    Eigen::Vector3f v2{ 0.0f, 0.0f, 0.0f };

    try {
        angleBetweenVectors( v1, v2 );
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ){
        EXPECT_EQ( err.what(), std::string( "Vector may not be zero length") );
    }
    catch( ... ) {
        FAIL( ) <<"Expected std::invalid_argument";
    }
}
 
TEST(TestAngleBetweenVectors, IdenticalVectorsShouldReturnZero) {
    Eigen::Vector3f v1{ 0.1f, 0.2f, 0.3f };
    Eigen::Vector3f v2{ 0.1f, 0.2f, 0.3f };

    float expected = 0.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

//
// Z Plane 
//

// 45 Degrees
TEST(TestAngleBetweenVectors, Test45DegreesShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 1.0f, 1.0f, 0.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 90 Degrees
TEST(TestAngleBetweenVectors, Test90DegreesShouldReturnPIOverTwo) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 0.0f, 3.0f, 0.0f };

    float expected = M_PI / 2.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 135 Degrees
TEST(TestAngleBetweenVectors, Test135DegreesShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ -1.0f, 1.0f, 0.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 180 Degrees
TEST(TestAngleBetweenVectors, Test180DegreesShouldReturnPI) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ -2.0f, 0.0f, 0.0f };

    float expected = M_PI;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


// 225 Degrees
TEST(TestAngleBetweenVectors, Test225DegreesShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ -1.0f, -1.0f, 0.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 270 Degrees
TEST(TestAngleBetweenVectors, Test270DegreesShouldReturnPIOverTwo) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 0.0f, -1.0f, 0.0f };

    float expected = M_PI / 2.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 315 Degrees
TEST(TestAngleBetweenVectors, Test315DegreesShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 1.0f, -1.0f, 0.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 360 Degrees
TEST(TestAngleBetweenVectors, Test360DegreesShouldReturnZero) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 2.0f, 0.0f, 0.0f };

    float expected = 0.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


//
// XZ Plane 
//

// 45 Degrees
TEST(TestAngleBetweenVectors, Test45DegreesInYPlaneShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 1.0f, 0.0f, 1.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 90 Degrees
TEST(TestAngleBetweenVectors, Test90DegreesInYPlaneShouldReturnPIOverTwo) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 0.0f, 0.0f, 3.0f };

    float expected = M_PI / 2.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 135 Degrees
TEST(TestAngleBetweenVectors, Test135DegreesInYPlaneShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ -1.0f, 0.0f, 1.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


// 225 Degrees
TEST(TestAngleBetweenVectors, Test225DegreesInYPlaneShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ -1.0f, 0.0f, -1.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 270 Degrees
TEST(TestAngleBetweenVectors, Test270DegreesInYPlaneShouldReturnPIOverTwo) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 0.0f, 0.0f, -1.0f };

    float expected = M_PI / 2.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 315 Degrees
TEST(TestAngleBetweenVectors, Test315DegreesInYPlaneShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 1.0f, 0.0f, -1.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


//
// YZ Plane 
//

// 45 Degrees
TEST(TestAngleBetweenVectors, Test45DegreesInXPlaneShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 0.0f, 0.0f, 2.0f };
    Eigen::Vector3f v2{ 0.0f, 1.0f, 1.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 135 Degrees
TEST(TestAngleBetweenVectors, Test135DegreesInXPlaneShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 0.0f, 0.0f, 2.0f };
    Eigen::Vector3f v2{ 0.0f, 1.0f, -1.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 180 Degrees
TEST(TestAngleBetweenVectors, Test180DegreesInXPlaneShouldReturnPI) {
    Eigen::Vector3f v1{ 0.0f, 0.0f, 2.0f };
    Eigen::Vector3f v2{ 0.0f, 0.0f, -2.0f };

    float expected = M_PI;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


// 225 Degrees
TEST(TestAngleBetweenVectors, Test225DegreesInXPlaneShouldReturnThreePIOverFour) {
    Eigen::Vector3f v1{ 0.0f, 0.0f, 2.0f };
    Eigen::Vector3f v2{ 0.0f, -1.0f, -1.0f };

    float expected = 3 * M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


// 315 Degrees
TEST(TestAngleBetweenVectors, Test315DegreesInXPlaneShouldReturnPIOverFour) {
    Eigen::Vector3f v1{ 0.0f, 0.0f, 2.0f };
    Eigen::Vector3f v2{ 0.0f, -1.0f, 1.0f };

    float expected = M_PI / 4.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}

// 360 Degrees
TEST(TestAngleBetweenVectors, Test360DegreesInXPlaneShouldReturnZero) {
    Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
    Eigen::Vector3f v2{ 2.0f, 0.0f, 0.0f };

    float expected = 0.0f;
    float actual   = angleBetweenVectors( v1, v2 );

    EXPECT_FLOAT_EQ( expected, actual );
}


//
// Test the 4-RoSy smallestAngle code
//

// TEST(TestAngleBetweenVectors, ZeroLengthRoSyVectorsShouldThrow) { 
//     Eigen::Vector3f v1{ 0.1f, 0.2f, 0.3f };
//     Eigen::Vector3f v2{ 0.0f, 0.0f, 0.0f };

//     try {
//         int i=-1, j=-1;
//         smallestAngleBetween4RoSyVectors( v1, v2, i, j );
//         FAIL() << "Expected std::invalid_argument";
//     }
//     catch ( std::invalid_argument const & err ){
//         EXPECT_EQ( err.what(), std::string( "Vector may not be zero length") );
//     }
//     catch( ... ) {
//         FAIL( ) <<"Expected std::invalid_argument";
//     }
// }
 
// TEST(TestAngleBetweenVectors, IdenticalRoSyVectorsShouldReturnZero) {
//     Eigen::Vector3f v1{ 0.1f, 0.2f, 0.3f };
//     Eigen::Vector3f v2{ 0.1f, 0.2f, 0.3f };

//     int i=-1, j=-1;
//     float expected = 0.0f;
//     float actual   = smallestAngleBetween4RoSyVectors( v1, v2, i, j );

//     EXPECT_FLOAT_EQ( expected, actual );
//     EXPECT_EQ( 0, i );
//     EXPECT_EQ( 0, j );
// }

// //
// // Z Plane 
// //

// // 30 Degrees
// TEST(TestAngleBetweenVectors, Test30DegreeZPlaneRoSyShouldReturn_30_0_0) {
//     Eigen::Vector3f v1{ sqrt( 3.0f ), 0.0f, 0.0f };
//     Eigen::Vector3f v2{ sqrt( 3.0f ), 1.0f, 0.0f };

//     int i=-1, j=-1;
//     float expected = M_PI / 6;
//     float actual   = smallestAngleBetween4RoSyVectors( v1, v2, i, j );

//     EXPECT_FLOAT_EQ( expected, actual );
//     EXPECT_EQ( 0, i );
//     EXPECT_EQ( 0, j );
// }

// // 45 Degrees
// TEST(TestAngleBetweenVectors, Test45DegreeZPlaneRoSyShouldReturn_45_0_0) {
//     Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
//     Eigen::Vector3f v2{ 1.0f, 1.0f, 0.0f };

//     int i=-1, j=-1;
//     float expected = M_PI / 4.0f;
//     float actual   = smallestAngleBetween4RoSyVectors( v1, v2, i, j );

//     EXPECT_FLOAT_EQ( expected, actual );
//     EXPECT_EQ( 0, i );
//     EXPECT_EQ( 0, j );
// }

// // 60 Degrees
// TEST(TestAngleBetweenVectors, Test60DegreeZPlaneRoSyShouldReturn_60_1_0) {
//     Eigen::Vector3f v1{ 1.0f, 0.0f, 0.0f };
//     Eigen::Vector3f v2{ 1.0f, 2.0f, 0.0f };

//     int i=-1, j=-1;
//     float expected = M_PI / 4.0f;
//     float actual   = smallestAngleBetween4RoSyVectors( v1, v2, i, j );

//     EXPECT_FLOAT_EQ( expected, actual );
//     EXPECT_EQ( 1, i );
//     EXPECT_EQ( 0, j );
// }

// // 90 Degrees
// TEST(TestAngleBetweenVectors, Test60DegreeZPlaneRoSyShouldReturn_0_1_0) {
//     Eigen::Vector3f v1{ 2.0f, 0.0f, 0.0f };
//     Eigen::Vector3f v2{ 0.0f, 1.0f, 0.0f };

//     int i=-1, j=-1;
//     float expected = M_PI / 4.0f;
//     float actual   = smallestAngleBetween4RoSyVectors( v1, v2, i, j );

//     EXPECT_FLOAT_EQ( expected, actual );
//     EXPECT_EQ( 1, i );
//     EXPECT_EQ( 0, j );
// }

}





