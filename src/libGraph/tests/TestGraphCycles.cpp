#include "TestGraphCycles.h"

#include <Graph/Graph.h>
#include <GeomFileUtils/ObjFileParser.h>
#include <unordered_set>
#include <vector>

void TestGraphCycles::TearDown( ) {}

/*
  Test Graph


    1       2      3      4      5      6
		 o------o------o------o------o------o
     |\   /  \    /                     |
		 | \ /	  \  /                      |
		 |  o      o----------o------o------o 7
     | 11\     10         9      8
	   |    \               |      |
		 |     \           13 |      | 14
		 |   12 o-------------o------o
  15 o-------------------/      /
	    \            16          /
	     \____________o_________/
 */
 animesh::Graph<std::size_t,int> setup_test_graph( ) {
	animesh::Graph<std::size_t,int> test_graph;
	animesh::Graph<size_t, int>::GraphNode * gn[17];
 	for ( size_t idx = 1; idx <= 16; ++idx ) {
 		gn[idx] = test_graph.add_node( idx );
 	}
 	test_graph.add_edge(gn[1], gn[2], 1.0, 0);
 	test_graph.add_edge(gn[2], gn[3], 1.0, 0);
 	test_graph.add_edge(gn[3], gn[4], 1.0, 0);
 	test_graph.add_edge(gn[4], gn[5], 1.0, 0);
 	test_graph.add_edge(gn[5], gn[6], 1.0, 0);
 	test_graph.add_edge(gn[6], gn[7], 1.0, 0);
 	test_graph.add_edge(gn[7], gn[8], 1.0, 0);
 	test_graph.add_edge(gn[8], gn[9], 1.0, 0);
 	test_graph.add_edge(gn[9], gn[10], 1.0, 0);
 	test_graph.add_edge(gn[11], gn[12], 1.0, 0);
 	test_graph.add_edge(gn[12], gn[13], 1.0, 0);
 	test_graph.add_edge(gn[13], gn[14], 1.0, 0);
 	test_graph.add_edge(gn[14], gn[16], 1.0, 0);
 	test_graph.add_edge(gn[1], gn[11], 1.0, 0);
 	test_graph.add_edge(gn[2], gn[11], 1.0, 0);
 	test_graph.add_edge(gn[2], gn[10], 1.0, 0);
 	test_graph.add_edge(gn[3], gn[10], 1.0, 0);
 	test_graph.add_edge(gn[8], gn[14], 1.0, 0);
 	test_graph.add_edge(gn[9], gn[13], 1.0, 0);
 	test_graph.add_edge(gn[13], gn[15], 1.0, 0);
 	test_graph.add_edge(gn[1], gn[15], 1.0, 0);
 	test_graph.add_edge(gn[15], gn[16], 1.0, 0);
	return test_graph;
}

animesh::Graph<animesh::PointNormal::Ptr,int>
setup_object(const std::string& file_name, bool face_wise) {
  using namespace std;
  using namespace animesh;

  ObjFileParser parser;
  Graph<animesh::PointNormal::Ptr,int> object;
  pair<vector<PointNormal::Ptr>, multimap<size_t, size_t>> results = parser.parse_file(file_name, true, face_wise);
  vector<Graph<PointNormal::Ptr, int>::GraphNode*> gn2;
  for( auto pt : results.first) {
    gn2.push_back(object.add_node(pt));
  }
  for( auto e: results.second) {
    if( !object.has_edge(gn2[e.first], gn2[e.second])) {
      object.add_edge(gn2[e.first], gn2[e.second], 1.0, 0);
    }
  }
  return object;
}


void TestGraphCycles::SetUp( ) {
  using namespace animesh;
  using namespace std;

  m_test_graph = setup_test_graph();
}

TEST_F(TestGraphCycles, IdenticalPathsAreEqual) {
  using namespace std;

	animesh::Path p1{ vector<size_t>{ 0, 1, 10 }};
	animesh::Path p2{ vector<size_t>{ 0, 1, 10 }};

  EXPECT_EQ(p1, p2);
}

TEST_F(TestGraphCycles, IdenticalPathsAreEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 0, 1, 10 }};
	animesh::Path p2{ vector<size_t>{ 0, 1, 10 }};
	EXPECT_TRUE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, ShiftedPathsAreNotEqual) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 0, 1, 10 }};
	animesh::Path p2{ vector<size_t>{ 1, 10, 0 }};
	EXPECT_FALSE(p1 == p2);
}

TEST_F(TestGraphCycles, ShiftedPathsAreNotEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 0, 1, 10 }};
	animesh::Path p2{ vector<size_t>{ 1, 10, 0 }};
	EXPECT_FALSE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, IdenticalCyclesAreEqual) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 1, 2, 3, 1 }};
	animesh::Path p2{ vector<size_t>{ 1, 2, 3, 1 }};
	EXPECT_EQ(p1, p2);
}

TEST_F(TestGraphCycles, IdenticalCyclesAreEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 1, 2, 3, 1 }};
	animesh::Path p2{ vector<size_t>{ 1, 2, 3, 1 }};
	EXPECT_TRUE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, ReversedCyclesAreEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 1, 2, 3, 1 }};
	animesh::Path p2{ vector<size_t>{ 1, 3, 2, 1 }};
  EXPECT_TRUE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, ShiftedCyclesAreEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 2, 3, 1, 2 }};
	animesh::Path p2{ vector<size_t>{ 1, 3, 2, 1 }};
  EXPECT_TRUE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, ReversedShiftedCyclesAreEquivalent) {
	using namespace std;

	animesh::Path p1{ vector<size_t>{ 2, 1, 3, 2 }};
	animesh::Path p2{ vector<size_t>{ 1, 3, 2, 1 }};
  EXPECT_TRUE(p1.is_equivalent_to(p2));
}

TEST_F(TestGraphCycles, SphereCycleCountIs100) {
  // Load the sphere
  animesh::Graph<animesh::PointNormal::Ptr,int> sphere10x10;
  sphere10x10 = setup_object("../data/sphere10x10/sphere10x10.obj", false);

	EXPECT_EQ( 100, sphere10x10.cycles().size());
}

TEST_F(TestGraphCycles, SphereFaceCycleCountIs92) {
  // Load the sphere
  animesh::Graph<animesh::PointNormal::Ptr,int> sphere10x10;
  sphere10x10 = setup_object("../data/sphere10x10/sphere10x10.obj", true);

	EXPECT_EQ( 92, sphere10x10.cycles().size());
}

TEST_F(TestGraphCycles, SphereFaceCycleCountIs22) {
  // Load the sphere
  animesh::Graph<animesh::PointNormal::Ptr,int> sphere5x5;
  sphere5x5 = setup_object("../data/Sphere5x5/sphere.obj", true);

	EXPECT_EQ( 22, sphere5x5.cycles().size());
}

// 0,1,2,3,0     0,1,5,4,0
// 0,3,7,4,0     6,2,3,7,6
// 6,2,1,5,6     6,7,4,5,6
TEST_F(TestGraphCycles, CubeCycleCountIs6) {
  animesh::Graph<animesh::PointNormal::Ptr,int>  cube = setup_object("../data/Cube1x1/cube.obj", false);

	EXPECT_EQ( 6, cube.cycles().size());
}

TEST_F(TestGraphCycles, CubeFaceCycleCountIs8) {
  animesh::Graph<animesh::PointNormal::Ptr,int>  cube = setup_object("../data/Cube1x1/cube.obj", true);

  // 8 vertices
	EXPECT_EQ( 8, cube.cycles().size());
}

// 289 vertices
// 256 faces
// Should be one cycle per face in node mode
TEST_F(TestGraphCycles, ClothPlaneCycleCountIs256) {
  animesh::Graph<animesh::PointNormal::Ptr,int>  cloth = setup_object("../data/Cloth Plane/cloth2_1.obj", false);

  // 256 vertices
  auto actual_cycles = cloth.cycles();
	EXPECT_EQ( 256, actual_cycles.size());
}

// 16x16 faces
// But each cycle covers 4 faces
// Should be 64 cyles
TEST_F(TestGraphCycles, ClothPlaneFaceCycleCountIs256) {
  animesh::Graph<animesh::PointNormal::Ptr,int>  cloth = setup_object("../data/Cloth Plane/cloth2_1.obj", true);

  auto actual_cycles = cloth.cycles();
	EXPECT_EQ( 64, actual_cycles.size());
}

TEST_F(TestGraphCycles, CyclesAreCorrect ) {
	using namespace std;

	// Note cycles are index order, not node value order`
	vector<animesh::Path> expected_cycles = {
		animesh::Path{ vector<size_t>{ 0, 1, 10, 0 }},
		animesh::Path{ vector<size_t>{ 1, 2, 9, 1}},
		animesh::Path{ vector<size_t>{ 2, 3, 4, 5, 6, 7, 8, 9, 2}},
		animesh::Path{ vector<size_t>{ 7, 13, 12, 8, 7}},
		animesh::Path{ vector<size_t>{ 12, 13, 15, 14, 12}},
		animesh::Path{ vector<size_t>{ 0, 10, 11, 12, 14, 0}},
	};

	vector<animesh::Path> cycles = m_test_graph.cycles( );
	EXPECT_EQ( expected_cycles.size(), cycles.size());

	for( auto expected_cycle : expected_cycles ) {
		// Find it
		bool found = false;
		for( auto actual_cycle : cycles) {
			if( actual_cycle.length() != expected_cycle.length() ) {
				continue;
			}
			found = (expected_cycle.is_equivalent_to(actual_cycle));
			if( found ) {
				break;
			}
		}
		EXPECT_EQ( true, found);
	}
}
