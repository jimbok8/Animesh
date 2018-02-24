#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <Graph/Graph.h>
#include <Graph/GraphNode.h>

#include "MockEdgeManager.h"
#include "TestGraph.h"

using ::testing::_;

namespace {

    /* **********************************************************************
     * *                                                                    *
     * * Graph Constructor tests                                            *
     * *                                                                    *
     * **********************************************************************/

    TEST(TestGraph, NullEdgeManagerShouldThrow) { 
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
    

    TEST(TestGraph, AddElementShouldMakeSizeLargerByOne) { 
    	MockEdgeManager em; 
    	Graph graph{ &em };

    	std::size_t old_size = graph.size();
    	std::size_t expected_size = old_size + 1;
        Eigen::Vector3f location{ 1.0f, 2.0f, 3.0f };
        Eigen::Vector3f   normal{ 1.0f, 0.0f, 0.0f };
        Element element{ location, normal };

        graph.addElement( element );

    	std::size_t actual_size = graph.size();
    	EXPECT_EQ( expected_size, actual_size );
    }

     /* **********************************************************************
      * *                                                                    *
      * *  Add Elements                                                      *
      * *                                                                    *
      * **********************************************************************/
    TEST(TestGraph, AddElementShouldCallEdgeManager) { 
    	MockEdgeManager em; 
    	Graph graph{ &em };

      Eigen::Vector3f location{ 1.0f, 2.0f, 3.0f };
      Eigen::Vector3f   normal{ 1.0f, 0.0f, 0.0f };
      Element element{ location, normal };

  		EXPECT_CALL( em, performEdgeManagement( _, _) ).Times( 1 ); 

      graph.addElement( element );
    }
}
