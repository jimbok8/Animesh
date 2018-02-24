#include "TestFieldData.h"

#include <Eigen/Core>

#include <Field/Field.h>
#include <Element/Element.h>


void TestFieldData::SetUp( ) {};
void TestFieldData::TearDown( ) {};


/* **********************************************************************
 * *                                                                    *
 * * FieldData Constructor tests                                            *
 * *                                                                    *
 * **********************************************************************/

TEST_F(TestFieldData, TangentVectorIsPerpendicularToNormal) { 
	Eigen::Vector3f	location{ 1.0f, 1.0f, 1.f };
	Eigen::Vector3f	normal  { 0.0f, 1.0f, 0.f };
	Element el_1_1_1{ location, normal };

    FieldData fieldData{ el_1_1_1 };

    Eigen::Vector3f tangent = fieldData.tangent( );

    EXPECT_FLOAT_EQ( 0.0f, tangent.dot( normal ) );
}

TEST_F(TestFieldData, TangentVectorIsUnit) { 
	Eigen::Vector3f	location{ 1.0f, 1.0f, 1.f };
	Eigen::Vector3f	normal  { 0.0f, 1.0f, 0.f };
	Element el_1_1_1{ location, normal };

    FieldData fieldData{ el_1_1_1 };

    Eigen::Vector3f tangent = fieldData.tangent( );

    EXPECT_FLOAT_EQ( 1.0f, tangent.norm() );
}

TEST_F(TestFieldData, KIsZero) { 
	Eigen::Vector3f	location{ 1.0f, 1.0f, 1.f };
	Eigen::Vector3f	normal  { 0.0f, 1.0f, 0.f };
	Element el_1_1_1{ location, normal };

    FieldData fieldData{ el_1_1_1 };

    EXPECT_FLOAT_EQ( 0.0f, fieldData.k( ) );
}
