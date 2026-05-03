//
// Created by gerrit on 3/11/25.
//

#pragma once
#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <unordered_map>
#include <vector>
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/diffgeo/tetrahedral.h"


namespace nonlinear_elasticity::collisions {

    inline long int hash(int x, int y, int z)
    {
        return (x * 18397) + (y * 20483) + (z * 29303);
    }


class CollisionDetection {
    public:
        CollisionDetection() = default;

        void init(const volumetric_base::LayeredMesh & undeformed_mesh);
        std::vector<std::tuple<volumetric_base::LayerVertex, volumetric_base::Prism, Eigen::Vector4d, Eigen::Vector4i>>
        test_for_prism_collision_candidates(
            const volumetric_base::LayeredMesh &mesh);
        std::vector<std::tuple<pmp::Vertex, volumetric_base::Tetrahedron, volumetric_base::Prism>> test_for_collisions(const volumetric_base::LayeredMesh &mesh, pmp::SurfaceMesh &hires) const;
        bool check_exact_collision(const Eigen::Vector3d &position,
                                   int prism_idx, const Eigen::MatrixXd &posis,
                                   Eigen::Vector4d &bary_coordinates,
                                   Eigen::Vector4i &bary_indices) const;


        std::vector<std::tuple<pmp::Vertex, std::array<int,4>>> test_for_collisions(const pmp::SurfaceMesh & hires, std::vector<std::array<int, 4>> tets, const Eigen::MatrixXd &points) const;
        std::pmr::unordered_map<long int, std::vector<volumetric_base::LayerVertex>> hash_grid;
        std::vector<volumetric_base::Tetrahedron> tets;
        std::vector<std::tuple<volumetric_base::LayerVertex, volumetric_base::Tetrahedron, volumetric_base::Prism>> colliding_tets;
        std::vector<double> tet_volume_signs;
        double cellsize =0;

};

}

#endif //COLLISIONS_H
