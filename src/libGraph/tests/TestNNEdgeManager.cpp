#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Graph/Graph.h>
#include <Graph/GraphNode.h>
#include <Element/Element.h>
#include <Graph/NNEdgeManager.h>
#include "TestNNEdgeManager.h"

using ::testing::_;

void TestNNEdgeManager::SetUp( ) {
    gn_origin = new GraphNode{ el_origin };
    gn_1_1_1  = new GraphNode{ el_1_1_1 };
    gn_1_0_0  = new GraphNode{ el_1_0_0 };
    gn_1_1_2  = new GraphNode{ el_1_1_2 };
    gn_1_1_3  = new GraphNode{ el_1_1_3 };
    gn_1_1_4  = new GraphNode{ el_1_1_4 };
    gn_1_1_5  = new GraphNode{ el_1_1_5 };
}


void TestNNEdgeManager::TearDown( ) {
    delete gn_origin;
    delete gn_1_1_1;
    delete gn_1_0_0;
    delete gn_1_1_2;
    delete gn_1_1_3;
    delete gn_1_1_4;
    delete gn_1_1_5;
}



    /* **********************************************************************
     * *                                                                    *
     * * Adding new edges.                                                  *
     * *                                                                    *
     * **********************************************************************/

    TEST_F(TestNNEdgeManager, insertNodeInEmptyListShouldInsertANode) { 
    	NNEdgeManager edgeManager{3};

    	// Insert
    	edgeManager.insertNodeInList( gn_1_0_0, gn_origin );

    	// Expect node was added to list
    	EXPECT_EQ( 1, gn_origin->edges().size() );
    }


    TEST_F(TestNNEdgeManager, insertNodeInEmptyListShouldInsertTheRightNode) { 
    	NNEdgeManager edgeManager{3};

    	// Insert
    	edgeManager.insertNodeInList( gn_1_0_0, gn_origin );

    	EXPECT_EQ( gn_1_0_0, gn_origin->edges().front()->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insertSmallestNodeInListShouldInsertTheNodeAtTheStart) { 
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_2, 1.0f, nullptr) );
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_4, 1.0f, nullptr) );

    	// Make existing closest to base
    	Eigen::Vector3f closest{ 1.0f, 0.5f, 1.0f };
    	Element el_closest{ closest, normal };
    	GraphNode * gn_closest = new GraphNode{ el_closest };

    	NNEdgeManager edgeManager{3};

    	// Insert
    	edgeManager.insertNodeInList( gn_closest, gn_1_1_1 );

    	EXPECT_EQ( gn_closest, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[1]->dest_node() );
    	EXPECT_EQ( gn_1_1_4, gn_1_1_1->edges()[2]->dest_node() );

    	delete gn_closest;
    }


    TEST_F(TestNNEdgeManager, insertLargestNodeInListShouldInsertTheNodeAtTheEnd) { 
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_2, 1.0f, nullptr ) );
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_4, 1.0f, nullptr ) );


    	// Make existing furthest to base
    	Eigen::Vector3f furthest{ 15.0f, 1.0f, 1.0f };
    	Element el_furthest{ furthest, normal };
    	GraphNode * gn_furthest = new GraphNode{ el_furthest };


    	NNEdgeManager edgeManager{3};

    	// Insert
    	edgeManager.insertNodeInList( gn_furthest, gn_1_1_1);

    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_4, gn_1_1_1->edges()[1]->dest_node() );
    	EXPECT_EQ( gn_furthest, gn_1_1_1->edges()[2]->dest_node() );

    	delete gn_furthest;
    }

    TEST_F(TestNNEdgeManager, insertMiddleNodeInListShouldInsertTheNodeInTheMiddle) { 
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_2, 1.0f, nullptr) );
    	gn_1_1_1->edges().push_back( new Edge( gn_1_1_1, gn_1_1_4, 1.0f, nullptr ) );


    	// Make existing new_node to base
    	Eigen::Vector3f new_node{ 1.0f, 3.0f, 1.0f };
    	Element el_new_node{ new_node, normal };
    	GraphNode * gn_new_node = new GraphNode{ el_new_node };

    	NNEdgeManager edgeManager{3};

    	// Insert
    	edgeManager.insertNodeInList( gn_new_node, gn_1_1_1 );

    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[1]->dest_node() );
    	EXPECT_EQ( gn_1_1_4, gn_1_1_1->edges() [2]->dest_node() );


    	delete gn_new_node;
    }

    /* **********************************************************************
     * *                                                                    *
     * * Manage edges from node                                             *
     * *                                                                    *
     * **********************************************************************/
    TEST_F(TestNNEdgeManager, insertNewNodeToFullNodeRemovesFurthest) { 

    	NNEdgeManager edgeManager{2};

    	gn_1_1_1->edges( ).push_back( new Edge( gn_1_1_1, gn_1_1_2, 1.0f, nullptr) );
    	gn_1_1_1->edges( ).push_back( new Edge( gn_1_1_1, gn_1_1_4, 1.0f, nullptr) );

    	// Insert
        GraphNode * gn_new_node = new GraphNode( el_1_1_3 );
    	edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node );

    	EXPECT_EQ( 2, gn_1_1_1->edges().size() );
    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[1]->dest_node() );

    	delete gn_new_node;
    }

    TEST_F(TestNNEdgeManager, insertNewNodeToFullNodeRemovesFurthestAddClosest) { 
    	// Make existing new_node to base
    	Eigen::Vector3f new_node{ 1.0f, 1.5f, 1.0f };
    	Element el_new_node{ new_node, normal };
    	GraphNode * gn_new_node = new GraphNode{ el_new_node };

    	NNEdgeManager edgeManager{2};

    	gn_1_1_1->edges( ).push_back( new Edge( gn_1_1_1, gn_1_1_2, 1.0f, nullptr) );
    	gn_1_1_1->edges( ).push_back( new Edge( gn_1_1_1, gn_1_1_4, 1.0f, nullptr) );


    	// Insert
    	edgeManager.manageEdgesFromNode( gn_1_1_1, gn_new_node);

    	EXPECT_EQ( gn_new_node, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[1]->dest_node() );
    	EXPECT_EQ( 2, gn_1_1_1->edges().size() );

    	delete gn_new_node;
    }


    /* **********************************************************************
     * *                                                                    *
     * * Check that node insertion works well                               *
     * *                                                                    *
     * **********************************************************************/


	TEST_F(TestNNEdgeManager, insertNewNodeMakesEachItsNeghbour ) { 
		NNEdgeManager edgeManager{2};
		std::vector<GraphNode *> all_nodes;
    	edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
    	all_nodes.push_back( gn_1_1_1 );
    	edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
    	all_nodes.push_back( gn_1_1_5 );

    	// Assert both nodes have one neighbour and it's the other node
    	EXPECT_EQ( 1, all_nodes[0]->edges().size() );
    	EXPECT_EQ( 1, all_nodes[1]->edges().size() );
        EXPECT_EQ( gn_1_1_5, all_nodes[0]->edges()[0]->dest_node() );
        EXPECT_EQ( gn_1_1_1, all_nodes[1]->edges()[0]->dest_node() );
    }



    TEST_F(TestNNEdgeManager, insert_113_Updates_111 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );


    	edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
    	all_nodes.push_back( gn_1_1_3 );


    	EXPECT_EQ( 2, gn_1_1_1->edges().size() );
    	EXPECT_EQ( gn_1_1_3, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_5, gn_1_1_1->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_113_Updates_115 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );

        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );

    	EXPECT_EQ( 2, gn_1_1_5->edges().size() );
    	EXPECT_EQ( gn_1_1_3, gn_1_1_5->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_1, gn_1_1_5->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_113_Updates_113 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );

        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );

    	EXPECT_EQ( 2, gn_1_1_3->edges().size() );
    	EXPECT_EQ( gn_1_1_1, gn_1_1_3->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_5, gn_1_1_3->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_112_updates_111 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );
        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );


    	edgeManager.performEdgeManagement( gn_1_1_2, all_nodes );
    	all_nodes.push_back( gn_1_1_2 );

    	EXPECT_EQ( 2, gn_1_1_1->edges().size() );
    	EXPECT_EQ( gn_1_1_2, gn_1_1_1->edges()[0]->dest_node() );
    	EXPECT_EQ( gn_1_1_3, gn_1_1_1->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_112_updates_113 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );
        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );

        edgeManager.performEdgeManagement( gn_1_1_2, all_nodes );
        all_nodes.push_back( gn_1_1_2 );

        EXPECT_EQ( 2, gn_1_1_3->edges().size() );
        EXPECT_EQ( gn_1_1_2, gn_1_1_3->edges()[0]->dest_node() );
        EXPECT_EQ( gn_1_1_1, gn_1_1_3->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_112_updates_115 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );
        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );

        edgeManager.performEdgeManagement( gn_1_1_2, all_nodes );
        all_nodes.push_back( gn_1_1_2 );

        EXPECT_EQ( 2, gn_1_1_5->edges().size() );
        EXPECT_EQ( gn_1_1_3, gn_1_1_5->edges()[0]->dest_node() );
        EXPECT_EQ( gn_1_1_2, gn_1_1_5->edges()[1]->dest_node() );
    }


    TEST_F(TestNNEdgeManager, insert_112_updates_112 ) { 
        NNEdgeManager edgeManager{2};
        std::vector<GraphNode *> all_nodes;
        edgeManager.performEdgeManagement( gn_1_1_1, all_nodes );
        all_nodes.push_back( gn_1_1_1 );
        edgeManager.performEdgeManagement( gn_1_1_5, all_nodes );
        all_nodes.push_back( gn_1_1_5 );
        edgeManager.performEdgeManagement( gn_1_1_3, all_nodes );
        all_nodes.push_back( gn_1_1_3 );

        edgeManager.performEdgeManagement( gn_1_1_2, all_nodes );
        all_nodes.push_back( gn_1_1_2 );

        EXPECT_EQ( 2, gn_1_1_2->edges().size() );
        EXPECT_EQ( gn_1_1_1, gn_1_1_2->edges()[0]->dest_node() );
        EXPECT_EQ( gn_1_1_3, gn_1_1_2->edges()[1]->dest_node() );
    }
