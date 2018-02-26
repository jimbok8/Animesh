#pragma once

#include "gtest/gtest.h"
#include <Eigen/Core>
#include <Field/Field.h>
#include <Element/Element.h>


class TestMatlabFieldExporter : public ::testing::Test {
public:
	void SetUp( ) override;
    void TearDown( ) override;
    	Eigen::Vector3f normal{ 0.0f, 1.0f, 0.0f };
    	Eigen::Vector3f point_1_1_1{ 1.0f, 1.0f, 1.0f };
    	Eigen::Vector3f point_1_1_2{ 1.0f, 1.0f, 2.0f };
    	Eigen::Vector3f point_1_1_3{ 1.0f, 1.0f, 3.0f };
    	Eigen::Vector3f point_2_1_1{ 2.0f, 1.0f, 1.0f };
    	Eigen::Vector3f point_2_1_2{ 2.0f, 1.0f, 2.0f };
    	Eigen::Vector3f point_2_1_3{ 2.0f, 1.0f, 3.0f };
    	Eigen::Vector3f point_3_1_1{ 3.0f, 1.0f, 1.0f };
    	Eigen::Vector3f point_3_1_2{ 3.0f, 1.0f, 2.0f };
    	Eigen::Vector3f point_3_1_3{ 3.0f, 1.0f, 3.0f };

    	Element el_1_1_1 { point_1_1_1, normal };
    	Element el_1_1_2 { point_1_1_2, normal };
    	Element el_1_1_3 { point_1_1_3, normal };
    	Element el_2_1_1 { point_2_1_1, normal };
    	Element el_2_1_2 { point_2_1_2, normal };
    	Element el_2_1_3 { point_2_1_3, normal };
    	Element el_3_1_1 { point_3_1_1, normal };
    	Element el_3_1_2 { point_3_1_2, normal };
    	Element el_3_1_3 { point_3_1_3, normal };
};