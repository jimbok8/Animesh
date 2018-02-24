#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Graph/Graph.h>

#include "MockEdgeManager.h"
#include "TestGraph.h"

void TestGraph::SetUp( ) {}
void TestGraph::TearDown( ) {}


    /* **********************************************************************
     * *                                                                    *
     * * Graph Constructor tests                                            *
     * *                                                                    *
     * **********************************************************************/

    TEST_F(TestGraph, NullEdgeManagerShouldThrow) { 
        try {
            Graph graph{ nullptr };
            FAIL() << "Expected std::invalid_argument";
        }
        catch ( std::invalid_argument const & err ) {
            EXPECT_EQ( err.what(), std::string( "EdgeManager may not be null") );
        }
        catch ( ... ) {
            FAIL( ) << "Expected std::invalid_argument";
        }
    }
    

    TEST_F(TestGraph, AddElementShouldMakeSizeLargerByOne) { 
    	MockEdgeManager em; 
    	Graph graph{ &em };

    	std::size_t old_size = graph.size();
    	std::size_t expected_size = old_size + 1;

      graph.addElement( el_1_1_1 );

    	std::size_t actual_size = graph.size();
    	EXPECT_EQ( expected_size, actual_size );
    }

     /* **********************************************************************
      * *                                                                    *
      * *  Add Elements                                                      *
      * *                                                                    *
      * **********************************************************************/
    TEST_F(TestGraph, AddElementShouldCallEdgeManager) { 
      using ::testing::_;


    	MockEdgeManager em; 
    	Graph graph{ &em };

  		EXPECT_CALL( em, performEdgeManagement( _, _) ).Times( 1 ); 

      graph.addElement( el_1_1_1 );
    }


    TEST_F(TestGraph, IteratorShouldWork ) { 
      MockEdgeManager em; 
      Graph graph{ &em };

      graph.addElement( el_1_1_1 );
      graph.addElement( el_1_1_2 );
      graph.addElement( el_1_1_3 );

      auto iter = graph.begin();
        // Should be three elements
      EXPECT_EQ( el_1_1_1, (*iter)->element() );
      ++iter;
      EXPECT_EQ( el_1_1_2, (*iter)->element() );
      ++iter;
      EXPECT_EQ( el_1_1_3, (*iter)->element() );
      ++iter;
      EXPECT_EQ( graph.end(), iter );
    }
