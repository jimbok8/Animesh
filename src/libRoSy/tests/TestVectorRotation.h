#include "gtest/gtest.h"
#include <RoSy/RoSy.h>

#include <Eigen/Core>

#pragma once

class TestVectorRotation : public ::testing::Test {
public:
    Eigen::Vector3f zero{ 0.0f, 0.0f, 0.0f };
    Eigen::Vector3f vec_1_0_0{ 1.0f, 0.0f, 0.0f };
    Eigen::Vector3f vec_0_1_0{ 0.0f, 1.0f, 0.0f };
    Eigen::Vector3f vec_0_0_1{ 0.0f, 0.0f, 1.0f };
    Eigen::Vector3f vec_1_1_1{ 1.0f, 1.0f, 1.0f };

    Eigen::Vector3f vec_2_1_0{ 2.0f, 1.0f, 0.0f };
    Eigen::Vector3f vec_2_0_1{ 2.0f, 0.0f, 1.0f };
    Eigen::Vector3f vec_0_2_1{ 0.0f, 2.0f, 1.0f };

    void SetUp( );
    void TearDown();
};