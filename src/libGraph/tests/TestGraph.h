#include <Element/Element.h>
#include <Eigen/Core>


class TestGraph : public ::testing::Test {
public:
	    Eigen::Vector3f origin{ 0.0f, 0.0f, 0.0f };
    	Eigen::Vector3f normal{ 0.0f, 1.0f, 0.0f };
    	Eigen::Vector3f point_1_1_1{ 1.0f, 1.0f, 1.0f };
    	Eigen::Vector3f point_1_1_2{ 1.0f, 1.0f, 2.0f };
    	Eigen::Vector3f point_1_1_3{ 1.0f, 1.0f, 3.0f };
    	Eigen::Vector3f point_1_1_4{ 1.0f, 1.0f, 4.0f };
    	Eigen::Vector3f point_1_1_5{ 1.0f, 1.0f, 5.0f };

    	Element el_origin{  origin, normal };
    	Element el_1_1_1 {  point_1_1_1, normal };
    	Element el_1_1_2 {  point_1_1_2, normal };
    	Element el_1_1_3 {  point_1_1_3, normal };
    	Element el_1_1_4 {  point_1_1_4, normal };
    	Element el_1_1_5 {  point_1_1_5, normal };

    	void SetUp( );
    	void TearDown( );
};