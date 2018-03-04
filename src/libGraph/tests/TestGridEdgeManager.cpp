#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Graph/Graph.h>
#include <Graph/GraphNode.h>
#include <Element/Element.h>

#include <Graph/GridEdgeManager.h>
#include "TestGridEdgeManager.h"

using ::testing::_;

void TestGridEdgeManager::SetUp( ) {
    gn_1_1_1  = new GraphNode{ el_1_1_1 };
    gn_2_1_1  = new GraphNode{ el_2_1_1 };
    gn_1_1_2  = new GraphNode{ el_1_1_2 };
    gn_1_2_1  = new GraphNode{ el_1_2_1 };
    gn_2_2_1  = new GraphNode{ el_2_2_1 };
    gn_2_1_2  = new GraphNode{ el_2_1_2 };
    gn_1_2_2  = new GraphNode{ el_1_2_2 };
}


void TestGridEdgeManager::TearDown( ) {
    delete gn_1_1_1;
    delete gn_2_1_1;
    delete gn_1_1_2;
    delete gn_1_2_1;
    delete gn_1_2_2;
    delete gn_2_1_2;
    delete gn_2_2_1;
}


/* **********************************************************************
 * *                                                                    *
 * * Manage edges from node                                             *
 * *                                                                    *
 * **********************************************************************/
TEST_F(TestGridEdgeManager, newNodeAtXGridSpacingShouldBeInserted) { 

	GridEdgeManager edgeManager{ 1.0f };

	// Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_1 );
	edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

	EXPECT_EQ( 1, gn_1_1_1->neighbours().size() );
	EXPECT_EQ( gn_new_node, gn_1_1_1->neighbours()[0] );

	EXPECT_EQ( 1, gn_new_node->neighbours().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->neighbours()[0] );

	delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanXGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanXGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeAtYGridSpacingShouldBeInserted) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 1, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( gn_new_node, gn_1_1_1->neighbours()[0] );

    EXPECT_EQ( 1, gn_new_node->neighbours().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->neighbours()[0] );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanYGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanYGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeAtZGridSpacingShouldBeInserted) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 1, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( gn_new_node, gn_1_1_1->neighbours()[0] );

    EXPECT_EQ( 1, gn_new_node->neighbours().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->neighbours()[0] );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanZGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanZGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeWithXAndYDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}
TEST_F(TestGridEdgeManager, newNodeWithXAndZDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}
TEST_F(TestGridEdgeManager, newNodeWithYAndZDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->neighbours().size() );
    EXPECT_EQ( 0, gn_new_node->neighbours().size() );

    delete gn_new_node;
}
