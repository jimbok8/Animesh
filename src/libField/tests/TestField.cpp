#include "TestField.h"

#include <Graph/Graph.h>
#include <Field/Field.h>


void TestField::SetUp( ) {}
void TestField::TearDown( ) {}


/* **********************************************************************
 * *                                                                    *
 * * Field Constructor tests                                            *
 * *                                                                    *
 * **********************************************************************/

TEST_F(TestField, CanIterateOverAllElements) { 
        Graph graph{ new NNEdgeManager{ 3} };
        graph.addElement( el_1_1_1 );
        graph.addElement( el_1_1_2 );
        graph.addElement( el_1_1_3 );

        Field field{ &graph };
}
