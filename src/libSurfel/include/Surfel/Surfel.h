//
// Created by Dave Durbin on 19/5/20.
//

#ifndef ANIMESH_SURFEL_H
#define ANIMESH_SURFEL_H

#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "FrameData.h"
#include <memory>

struct Surfel {
    Surfel(std::string id,
           const std::vector<FrameData> &frames,
           Eigen::Vector3f tangent,
           Eigen::Vector3f closest_mesh_vertex_position
    );

    static std::map<std::string, std::shared_ptr<Surfel>> surfel_by_id;

    static std::shared_ptr<Surfel> surfel_for_id(const std::string &id);

    std::string id;
    std::vector<FrameData> frame_data;
    Eigen::Vector3f tangent;
    // Relative position of representation lattice intersection [0,1)
    Eigen::Vector3f closest_mesh_vertex_position;
    float last_correction;
    float error;
    float position_error;
};

#endif //ANIMESH_SURFEL_H
