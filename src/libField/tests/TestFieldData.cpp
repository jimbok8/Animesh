#include "TestFieldData.h"

#include <Element/Element.h>

#include "gmock/gmock.h"

void TestFieldData::SetUp( ) {};
void TestFieldData::TearDown( ) {};

const float EPSILON = 1e-6;

/* **********************************************************************
 * *                                                                    *
 * * FieldData Constructor tests                                            *
 * *                                                                    *
 * **********************************************************************/

TEST_F(TestFieldData, TangentVectorIsPerpendicularToNormal) { 
    Eigen::Vector3f tangent = field_data_1_1_1.tangent( );

    EXPECT_FLOAT_EQ( 0.0f, tangent.dot( normal ) );
}

TEST_F(TestFieldData, TangentVectorIsUnit) { 
    Eigen::Vector3f tangent = field_data_1_1_1.tangent( );

    EXPECT_FLOAT_EQ( 1.0f, tangent.norm() );
}

TEST_F(TestFieldData, KIsZero) { 
    EXPECT_FLOAT_EQ( 0.0f, field_data_1_1_1.k( ) );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_000 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_000, normal );

	EXPECT_NEAR( best[0], vector_000[0], EPSILON );
	EXPECT_NEAR( best[1], vector_000[1], EPSILON );
	EXPECT_NEAR( best[2], vector_000[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_030 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_030, normal );

	EXPECT_NEAR( best[0], vector_030[0], EPSILON );
	EXPECT_NEAR( best[1], vector_030[1], EPSILON );
	EXPECT_NEAR( best[2], vector_030[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_045 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_045, normal );

	EXPECT_NEAR( best[0], vector_045[0], EPSILON );
	EXPECT_NEAR( best[1], vector_045[1], EPSILON );
	EXPECT_NEAR( best[2], vector_045[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_060 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_060, normal );

	EXPECT_NEAR( best[0], vector_330[0], EPSILON );
	EXPECT_NEAR( best[1], vector_330[1], EPSILON );
	EXPECT_NEAR( best[2], vector_330[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_090 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_090, normal );

	EXPECT_NEAR( best[0], vector_000[0], EPSILON );
	EXPECT_NEAR( best[1], vector_000[1], EPSILON );
	EXPECT_NEAR( best[2], vector_000[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_120 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_120, normal );

	EXPECT_NEAR( best[0], vector_030[0], EPSILON );
	EXPECT_NEAR( best[1], vector_030[1], EPSILON );
	EXPECT_NEAR( best[2], vector_030[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_135 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_135, normal );

	EXPECT_NEAR( best[0], vector_315[0], EPSILON );
	EXPECT_NEAR( best[1], vector_315[1], EPSILON );
	EXPECT_NEAR( best[2], vector_315[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_150 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_150, normal );

	EXPECT_NEAR( best[0], vector_330[0], EPSILON );
	EXPECT_NEAR( best[1], vector_330[1], EPSILON );
	EXPECT_NEAR( best[2], vector_330[2], EPSILON );
}

TEST_F( TestFieldData, BestRosyForFieldDataIsCorrectFor_180 ) {
	Eigen::Vector3f best = best_rosy_vector_for( vector_000, normal, 0, vector_180, normal );

	EXPECT_NEAR( best[0], vector_000[0], EPSILON );
	EXPECT_NEAR( best[1], vector_000[1], EPSILON );
	EXPECT_NEAR( best[2], vector_000[2], EPSILON );
}