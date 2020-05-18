//
// Created by Dave Durbin on 18/5/20.
//

#include <PoSy/PoSyOptimiser.h>
#include "TestPoSyOptimiser.h"
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
    animesh::Graph<std::shared_ptr<Surfel>,int> g;
    optimiser.set_data(g);

    optimiser.optimise_do_one_step();
}
