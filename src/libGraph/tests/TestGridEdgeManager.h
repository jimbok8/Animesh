#include <Graph/GraphNode.h>
#include <Element/Element.h>
#include <Eigen/Core>

#include "gtest/gtest.h"


class TestGridEdgeManager : public ::testing::Test {
public:
    	Eigen::Vector3f normalXZ{ 0.0f, 1.0f, 0.0f };
    	Eigen::Vector3f normalXY{ 0.0f, 0.0f, 1.0f };

    	Eigen::Vector3f point_1_1_1{ 1.0f, 1.0f, 1.0f };
        Eigen::Vector3f point_2_1_1{ 2.0f, 1.0f, 1.0f };
        Eigen::Vector3f point_1_1_2{ 1.0f, 1.0f, 2.0f };
        Eigen::Vector3f point_1_2_1{ 1.0f, 2.0f, 1.0f };
        Eigen::Vector3f point_2_2_1{ 2.0f, 2.0f, 1.0f };
        Eigen::Vector3f point_1_2_2{ 1.0f, 2.0f, 2.0f };
        Eigen::Vector3f point_2_1_2{ 2.0f, 1.0f, 2.0f };

    	Element el_1_1_1 { point_1_1_1, normalXZ };
        Element el_2_1_1 { point_2_1_1, normalXZ };
        Element el_1_1_2 { point_1_1_2, normalXZ };
        Element el_1_2_1 { point_1_2_1, normalXY };
        Element el_2_2_1 { point_2_2_1, normalXY };
        Element el_1_2_2 { point_1_2_2, normalXY };
        Element el_2_1_2 { point_2_1_2, normalXY };

    	GraphNode * gn_1_1_1;
        GraphNode * gn_2_1_1;
        GraphNode * gn_1_1_2;
        GraphNode * gn_1_2_1;
        GraphNode * gn_2_2_1;
        GraphNode * gn_1_2_2;
        GraphNode * gn_2_1_2;

    	void SetUp( );
    	void TearDown( );
};