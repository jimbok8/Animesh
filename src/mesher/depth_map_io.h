//
// Created by Dave Durbin on 2019-08-21.
//

#ifndef ANIMESH_DEPTH_MAP_IO_H
#define ANIMESH_DEPTH_MAP_IO_H

#include <DepthMap/DepthMapPyramid.h>
#include "mesher_args.h"

/**
 * Load the depth maps according to the provided arguments.
 * @param args The arguments specifying how to load.
 * @param pyramids The vector to be populated with DephgMapPyramids
 */
void
load_depth_map_pyramids(MesherArguments &args, std::vector<DepthMapPyramid> &pyramids);

#endif //ANIMESH_DEPTH_MAP_IO_H
