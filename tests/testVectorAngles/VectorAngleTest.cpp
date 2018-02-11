#include "VectorAngleTest.h"

using ::testing::Return;


TEST_F(VectorAngleTest, ZeroLengthVectorsShouldThrow) { 
	Vector3f v1{ 0.1f, 0.2f, 0.3f };
	Vector3f v2{ 0.0f, 0.0f, 0.0f };

    ASSERT_FAIL();
}
 
TEST(VectorAngleTest, IdenticalVectorsShouldReturnZero) {
   ASSERT_FAIL( );
}




