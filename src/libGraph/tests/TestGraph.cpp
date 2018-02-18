#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Graph/Graph.h>
#include <Graph/GraphNode.h>

#include "MockEdgeManager.h"

using ::testing::_;

namespace {

    /* **********************************************************************
     * *                                                                    *
     * * Graph Constructor tests                                            *
     * *                                                                    *
     * **********************************************************************/

    TEST(TestGraph, NullEdgeManagerShouldThrow) { 
    	Graph graph{ nullptr };
    }

    TEST(TestGraph, AddElementShouldMakeSizeLargerByOne) { 
    	MockEdgeManager em; 
    	Graph graph{ &em };

    	std::size_t old_size = graph.size();

    	std::size_t expected_size = old_size + 1;


    	std::size_t actual_size = graph.size();

    	EXPECT_EQ( expected_size, actual_size );
    }

    TEST(TestGraph, AddElementShouldCallEdgeManager) { 
    	MockEdgeManager em; 
    	Graph graph{ &em };

		EXPECT_CALL(em, performEdgeManagement(_,_))
      					.Times(1);

    }
}