#pragma once

#include <Field/FieldData.h>
#include <Eigen/Core>
#include "gtest/gtest.h"

class TestFieldData : public ::testing::Test {
public:
	void SetUp( ) override;
    void TearDown( ) override;

	Eigen::Vector3f	normal  { 0.0f, 1.0f, 0.f };

    Eigen::Vector3f	point_1_1_1{ 1.0f, 1.0f, 1.f };

	Element el_1_1_1{ point_1_1_1, normal };

	FieldData field_data_1_1_1{ el_1_1_1 };

	float ROOT_3_OVER_2 = sqrt( 3.0f ) / 2.0f;
	float ONE_OVER_ROOT_2 = 1.0f / sqrt( 2.0f );
	Eigen::Vector3f vector_000{ 1.0f,            0.0f, 0.0f };
	Eigen::Vector3f vector_030{ ROOT_3_OVER_2,	 0.0f, 0.5f };
	Eigen::Vector3f vector_045{ ONE_OVER_ROOT_2, 0.0,  ONE_OVER_ROOT_2 };
	Eigen::Vector3f vector_060{ 0.5f,		   	 0.0f, ROOT_3_OVER_2 };
	Eigen::Vector3f vector_090{ 0.0f,            0.0f, 1.0f };
	Eigen::Vector3f vector_120{ -0.5f,  		 0.0f, ROOT_3_OVER_2 };
	Eigen::Vector3f vector_135{ -ONE_OVER_ROOT_2,0.0f, ONE_OVER_ROOT_2 };
	Eigen::Vector3f vector_150{ -ROOT_3_OVER_2,  0.0f, 0.5f };
	Eigen::Vector3f vector_180{ -1.0f, 			 0.0,  0.0f };
	Eigen::Vector3f vector_315{  ONE_OVER_ROOT_2,0.0f, -ONE_OVER_ROOT_2 };
	Eigen::Vector3f vector_330{  ROOT_3_OVER_2,  0.0f, -0.5f };
};