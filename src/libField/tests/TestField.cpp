#include "TestField.h"

#include <Graph/Graph.h>
#include <Field/Field.h>


void TestField::SetUp( ) {}
void TestField::TearDown( ) {}


/* **********************************************************************
 * *                                                                    *
 * * Graph Constructor tests                                            *
 * *                                                                    *
 * **********************************************************************/

TEST_F(TestField, TestGraphIsSetFromConstructor) { 
        Graph graph{ new NNEdgeManager{ 3} };
        Field field{ &graph };
        FAIL( );
}
