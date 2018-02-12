//#include "VectorAngleTest.h"
#include "gtest/gtest.h"
#include <Eigen/Core>


namespace {
TEST(VectorAngleTest, ZeroLengthVectorsShouldThrow) { 
	Eigen::Vector3f v1{ 0.1f, 0.2f, 0.3f };
	Eigen::Vector3f v2{ 0.0f, 0.0f, 0.0f };

    ASSERT_TRUE( false);
}
 
TEST(VectorAngleTest, IdenticalVectorsShouldReturnZero) {
   ASSERT_TRUE( false );
}

}




