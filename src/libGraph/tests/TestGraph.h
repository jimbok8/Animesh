#include <Graph/GraphNode.h>
#include <Element/Element.h>
#include <Eigen/Core>


class TestGraph : ::testing::Test {
public:
	    Eigen::Vector3f origin{ 0.0f, 0.0f, 0.0f };
    	Eigen::Vector3f normal{ 0.0f, 1.0f, 0.0f };
    	Eigen::Vector3f base  { 1.0f, 1.0f, 1.0f };
    	Eigen::Vector3f point_1_1_2{ 1.0f, 1.0f, 2.0f };
    	Eigen::Vector3f point_1_1_3{ 1.0f, 1.0f, 3.0f };
    	Eigen::Vector3f point_1_1_4{ 1.0f, 1.0f, 4.0f };
    	Eigen::Vector3f point_1_1_5{ 1.0f, 1.0f, 5.0f };

    	Element el_origin{  origin, normal };
    	Element el_base  {  origin, normal };
    	Element el_1_1_2 {  origin, point_1_1_2 };
    	Element el_1_1_3 {  origin, point_1_1_3 };
    	Element el_1_1_4 {  origin, point_1_1_4 };
    	Element el_1_1_5 {  origin, point_1_1_5 };

    	GraphNode * gn_origin;
    	GraphNode * gn_base;
    	GraphNode * gn_1_1_2;
    	GraphNode * gn_1_1_3;
    	GraphNode * gn_1_1_4;
    	GraphNode * gn_1_1_5;

    	void SetUp( );
    	void TearDown( );
};