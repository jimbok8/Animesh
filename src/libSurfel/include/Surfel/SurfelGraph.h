#pragma once

#include <Eigen/Core>
#include <Graph/Graph.h>
#include "Surfel.h"

struct SurfelGraphEdge {
    float weight;
    int rosy_ij;
    int rosy_ji;
    Eigen::Vector2i posy_tij;
    Eigen::Vector2i posy_tji;
    SurfelGraphEdge( float weight ) :
            weight{weight},
            rosy_ij{0},
            rosy_ji{0},
            posy_tij{Eigen::Vector2i::Zero()},
            posy_tji{Eigen::Vector2i::Zero()}
    {}
};
using SurfelGraph = animesh::Graph<std::shared_ptr<Surfel>, SurfelGraphEdge>;
using SurfelGraphNodePtr = std::shared_ptr<animesh::Graph<std::shared_ptr<Surfel>, SurfelGraphEdge>::GraphNode>;
