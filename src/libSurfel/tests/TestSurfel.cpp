#include "TestSurfel.h"
#include <Surfel/Surfel.h>
#include <Surfel/FrameData.h>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <memory>

void TestSurfel::SetUp( ) {}
void TestSurfel::TearDown( ) {}


/* ********************************************************************************
 * *
 * *  Test average rosy vectors
 * *   
 * ********************************************************************************/
TEST_F(TestSurfel, NeighboursAreCopied) {
    using namespace std;
    using namespace Eigen;

    vector<shared_ptr<Surfel>> neighbours;
    vector<string> neighbour_ids{"two", "three", "four"};

    Surfel s1{"one", vector<FrameData>{}, neighbours, Vector3f::Zero()};
    Surfel s2{"two", vector<FrameData>{}, neighbours, Vector3f::Zero()};
    Surfel s3{"three", vector<FrameData>{}, neighbours, Vector3f::Zero()};
    Surfel s4{"four", vector<FrameData>{}, neighbours, Vector3f::Zero()};

    s1.neighbouring_surfels.push_back(make_shared<Surfel>(s2));
    s1.neighbouring_surfels.push_back(make_shared<Surfel>(s3));
    s1.neighbouring_surfels.push_back(make_shared<Surfel>(s4));

    EXPECT_EQ(s1.neighbouring_surfels.size(), 3);
    auto it = neighbour_ids.begin();
    for( const auto& surfel_ptr : s1.neighbouring_surfels) {
        EXPECT_EQ(surfel_ptr->id, *it++);
    }
}