#include <gmock/gmock.h>
#include <Graph/EdgeManager.h>

class MockEdgeManager : public EdgeManager {
public:
	MOCK_METHOD2(performEdgeManagement,  void( GraphNode& newNode, std::vector<GraphNode> existingNodes) );
	
};