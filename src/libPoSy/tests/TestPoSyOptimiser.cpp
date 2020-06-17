//
// Created by Dave Durbin on 18/5/20.
//

#include <PoSy/PoSyOptimiser.h>
#include "TestPoSyOptimiser.h"
#include <memory>

void TestPoSyOptimiser::SetUp( ) {}
void TestPoSyOptimiser::TearDown( ) {}

/* ********************************************************************************
 * *
 * *  Test average rosy vectors
 * *
 * ********************************************************************************/
TEST_F(TestPoSyOptimiser, FailsAssertionOptimisingWhenUnready) {
    Properties p{};
    PoSyOptimiser optimiser{p};

    ASSERT_DEATH(optimiser.optimise_do_one_step(), "(m_state != UNINITIALISED)");
}

TEST_F(TestPoSyOptimiser, IsReadyOnceDataIsSet) {
    Properties p{};
    PoSyOptimiser optimiser{p};
    animesh::Graph<std::shared_ptr<Surfel>,float> g;
    optimiser.set_data(g);

    optimiser.optimise_do_one_step();
}

//TEST_F(TestPoSyOptimiser, ConvergesInPlane) {
//    using namespace std;
//    using namespace Eigen;
//
//    map<string, string> props = {
//            {"rho", "1.0"}
//    };
//    Properties p{props};
//    PoSyOptimiser optimiser{p};
//
//    animesh::Graph<shared_ptr<Surfel>,int> g;
//    auto a = make_shared<Surfel>("a", vector<FrameData>{},
//    vector<shared_ptr<Surfel>>{},
//    Eigen::Vector3f tangent));
//    auto b = make_ptr();
//    g.add_node(a);
//    g.add_node(b);
//    g.add_edge(a,b);
//    optimiser.set_data(g);
//    optimiser.optimise_do_one_step();
//}
