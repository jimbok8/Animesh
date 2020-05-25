//
// Created by Dave Durbin on 2019-07-06.
//
#pragma once

#ifndef ANIMESH_TYPES_H
#define ANIMESH_TYPES_H

#include <string>
#include <Eigen/Core>

struct SurfelInFrame {
    std::string surfel_id;
    size_t frame_index;

    SurfelInFrame(std::string s, size_t f) : surfel_id{std::move(s)}, frame_index{f} {}

    bool operator<(const SurfelInFrame &other) const {
        if (frame_index != other.frame_index)
            return frame_index < other.frame_index;

        return surfel_id < other.surfel_id;
    }
};

struct NormalTangent {
    Eigen::Vector3f normal;
    Eigen::Vector3f tangent;

    NormalTangent(Eigen::Vector3f n, Eigen::Vector3f t) : normal{std::move(n)}, tangent{std::move(t)} {
    }
};

#endif //ANIMESH_TYPES_H
