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

	EXPECT_EQ( 1, gn_1_1_1->edges().size() );
	EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[0]->dest_node() );

	EXPECT_EQ( 1, gn_new_node->edges().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->edges()[0]->dest_node() );

	delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanXGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanXGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeAtYGridSpacingShouldBeInserted) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 1, gn_1_1_1->edges().size() );
    EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[0]->dest_node() );

    EXPECT_EQ( 1, gn_new_node->edges().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->edges()[0]->dest_node() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanYGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanYGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeAtZGridSpacingShouldBeInserted) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 1, gn_1_1_1->edges().size() );
    EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[0]->dest_node() );

    EXPECT_EQ( 1, gn_new_node->edges().size() );
    EXPECT_EQ( gn_1_1_1, gn_new_node->edges()[0]->dest_node() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeFurtherThanZGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 0.5f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeCloserThanZGridSpacingShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 2.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}

TEST_F(TestGridEdgeManager, newNodeWithXAndYDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_2_1 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}
TEST_F(TestGridEdgeManager, newNodeWithXAndZDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_2_1_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}
TEST_F(TestGridEdgeManager, newNodeWithYAndZDeltasShouldNotBeInserted ) { 

    GridEdgeManager edgeManager{ 1.0f };

    // Insert
    GraphNode * gn_new_node = new GraphNode( el_1_2_2 );
    edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    EXPECT_EQ( 0, gn_1_1_1->edges().size() );
    EXPECT_EQ( 0, gn_new_node->edges().size() );

    delete gn_new_node;
}


TEST_F(TestGridEdgeManager, graphEdgeNodesHaveCadence3 ) { 
    using namespace Eigen;

    EdgeManager *em = new GridEdgeManager{1.0f};
    Graph *g = new Graph( em );

    for( int y = 0; y<5; y++ ) {
        for( int x = 0; x<5; x++ ) {

            Vector3f location { x, y, 0.0f };
            Vector3f normal{ 0.0f, 0.0f, 1.0f };
            Element *e = new Element( location, normal );

            g->addElement( *e );
        }
    }

    // Check
    int idx = 0;
    for( auto gi = g->begin(); gi != g->end(); ++gi ) {
        int actual_size = (*gi)->edges().size();

        if (   (idx == 1) || (idx == 2) || (idx == 3 )
            || (idx == 5 ) || (idx == 9)
            || (idx == 10) || (idx == 14)
            || (idx == 15) || (idx == 19)
            || (idx == 21) || (idx == 22) || (idx == 23) ) {

            EXPECT_EQ( 3, actual_size );
        }

        ++idx;
    }
}


TEST_F(TestGridEdgeManager, graphCornerNodesHaveCadence2 ) { 
    using namespace Eigen;

    EdgeManager *em = new GridEdgeManager{1.0f};
    Graph *g = new Graph( em );

    for( int y = 0; y<5; y++ ) {
        for( int x = 0; x<5; x++ ) {

            Vector3f location { x, y, 0.0f };
            Vector3f normal{ 0.0f, 0.0f, 1.0f };
            Element *e = new Element( location, normal );

            g->addElement( *e );
        }
    }

    // Check
    int idx = 0;
    for( auto gi = g->begin(); gi != g->end(); ++gi ) {
        int actual_size = (*gi)->edges().size();

        if( idx == 0 || idx == 4 || idx == 20 || idx == 24 ) {
            EXPECT_EQ( 2, actual_size );
        } 

        ++idx;
    }
}

TEST_F(TestGridEdgeManager, graphMidNodesHaveCadence4 ) { 
    using namespace Eigen;

    EdgeManager *em = new GridEdgeManager{1.0f};
    Graph *g = new Graph( em );

    for( int y = 0; y<5; y++ ) {
        for( int x = 0; x<5; x++ ) {

            Vector3f location { x, y, 0.0f };
            Vector3f normal{ 0.0f, 0.0f, 1.0f };
            Element *e = new Element( location, normal );

            g->addElement( *e );
        }
    }

    // Check
    int idx = 0;
    for( auto gi = g->begin(); gi != g->end(); ++gi ) {
        int actual_size = (*gi)->edges().size();

        if( ( idx > 4 ) && ( idx < 20 ) && ( ( idx % 5 ) != 0 ) && ( ( idx % 5 ) != 4 ) ) {
            EXPECT_EQ( 4, actual_size );
        } 

        ++idx;
    }
}