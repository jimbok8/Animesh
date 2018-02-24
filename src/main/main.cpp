#include <iostream>

#include <VectorAngle/VectorAngle.h>
#include <Graph/Graph.h>


int main( int argc, char * argv[] ) {

	EdgeManager *em = new NNEdgeManager{4};

	Graph g{ em };
	
    return 0;
}
