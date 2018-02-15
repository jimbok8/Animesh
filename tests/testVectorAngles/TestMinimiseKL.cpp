#include "gtest/gtest.h"
#include "angleBetweenVectors.h"

/*
 * Test the function to return k,l that minimise the inter vectr angle
 */

namespace {
/* ********************************************************************************
 * ** Test computing the smallest rotations
 * ********************************************************************************/

TEST( TestVectorRotation, ShouldThrowIfN1IsNotUnitVector ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 2.0f, 3.0f };
    Vector3f n1{ 1.0f, 1.0f, 0.0f };
    Vector3f o2{ 0.0f, 2.0f, 3.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    try {
        computeOptimalKL( o1, n1, o2, n2, actualK, actualL);
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Normal must be unit vector") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}

TEST( TestVectorRotation, ShouldThrowIfO1IsZero ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 0.0f, 0.0f };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 2.0f, 3.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    try {
        computeOptimalKL( o1, n1, o2, n2, actualK, actualL);
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Vector may not be zero length") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}

TEST( TestVectorRotation, ShouldThrowIfN2IsNotUnitVector ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 2.0f, 3.0f };
    Vector3f n1{ 1.0f, 1.0f, 0.0f };
    Vector3f o2{ 0.0f, 2.0f, 3.0f };
    Vector3f n2{ 1.0f, 1.0f, 0.0f };

    int actualK;
    int actualL;

    try {
        computeOptimalKL( o1, n1, o2, n2, actualK, actualL);
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Normal must be unit vector") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}

TEST( TestVectorRotation, ShouldThrowIfO2IsZero ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 2.0f, 3.0f };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 0.0f, 0.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    try {
        computeOptimalKL( o1, n1, o2, n2, actualK, actualL);
        FAIL() << "Expected std::invalid_argument";
    }
    catch ( std::invalid_argument const & err ) {
        EXPECT_EQ( err.what(), std::string( "Vector may not be zero length") );
    }
    catch ( ... ) {
        FAIL( ) << "Expected std::invalid_argument";
    }
}


/* ***
 * *
 * */
TEST( TestVectorRotation, ShouldBe_0_0_For_0_DegreesCoplanar ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 1.0f, 0.0f };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 1.0f, 0.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    int expectedK = 0;
    int expectedL = 0;
    float expectedTheta = 0.0f;

    float actualTheta = computeOptimalKL( o1, n2, o2, n2, actualK, actualL );

    EXPECT_EQ( expectedK, actualK );
    EXPECT_EQ( expectedL, actualL );
    EXPECT_FLOAT_EQ( expectedTheta, actualTheta );
}

TEST( TestVectorRotation, ShouldBe_0_0_For_30_DegreesCoplanar ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, sqrt( 3.0f ), 1.0f };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 1.0f, 0.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    int expectedK = 0;
    int expectedL = 0;
    float expectedTheta = M_PI / 6.0f;

    float actualTheta = computeOptimalKL( o1, n2, o2, n2, actualK, actualL );

    EXPECT_EQ( expectedK, actualK );
    EXPECT_EQ( expectedL, actualL );
    EXPECT_FLOAT_EQ( expectedTheta, actualTheta );
}

TEST( TestVectorRotation, ShouldBe_0_0_For_45_DegreesCoplanar ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 1.0f, 1.0f };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 1.0f, 0.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    int expectedK = 0;
    int expectedL = 0;
    float expectedTheta = M_PI / 4.0f;

    float actualTheta = computeOptimalKL( o1, n2, o2, n2, actualK, actualL );

    EXPECT_EQ( expectedK, actualK );
    EXPECT_EQ( expectedL, actualL );
    EXPECT_FLOAT_EQ( expectedTheta, actualTheta );
}


TEST( TestVectorRotation, ShouldBe_0_1_For_60_DegreesCoplanar ) {
    using namespace Eigen;

    Vector3f o1{ 0.0f, 1.0f, sqrt( 3.0f ) };
    Vector3f n1{ 1.0f, 0.0f, 0.0f };
    Vector3f o2{ 0.0f, 1.0f, 0.0f };
    Vector3f n2{ 1.0f, 0.0f, 0.0f };

    int actualK;
    int actualL;

    int expectedK = 0;
    int expectedL = 1;
    float expectedTheta = M_PI / 6.0f;

    float actualTheta = computeOptimalKL( o1, n2, o2, n2, actualK, actualL );

    EXPECT_EQ( expectedK, actualK );
    EXPECT_EQ( expectedL, actualL );
    EXPECT_FLOAT_EQ( expectedTheta, actualTheta );
}
}