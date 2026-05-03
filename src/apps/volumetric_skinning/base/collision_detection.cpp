#include "collision_detection.h"

#include "volumetric_base/diffgeo/prism_diffgeo.h"

#include <vector>


namespace nonlinear_elasticity::collisions {
    bool vertex_in_tet(const Eigen::PartialPivLU<Eigen::Matrix3d> &T, const Eigen::Vector3d &v) {
        const Eigen::Vector3d barycenter = T.solve(v);
        return barycenter.minCoeff() >= 0 && barycenter.sum() <= 1.0;
    }

    void CollisionDetection::init(const volumetric_base::LayeredMesh &undeformed_mesh) {
        Eigen::MatrixXd positions = undeformed_mesh.toVertexMatrix();
        tet_volume_signs = std::vector<double>(undeformed_mesh.n_prisms() * 3);
        double avg_len = 0;
        unsigned int anz = 0;
        const auto tets_and_cells = volumetric_base::tetrahedral::getTetrahedraAndCells(undeformed_mesh);
        for (unsigned int i = 0; i < tets_and_cells.size(); ++i) {
            auto [tet, cell] = tets_and_cells[i];
            const auto &[i1,i2,i3,i4] = tet;
            const Eigen::Vector3d &p1 = positions.row(i1);
            const Eigen::Vector3d &p2 = positions.row(i2);
            const Eigen::Vector3d &p3 = positions.row(i3);
            const Eigen::Vector3d &p4 = positions.row(i4);

            anz += 6;
            avg_len += (p1 - p2).norm();
            avg_len += (p1 - p3).norm();
            avg_len += (p1 - p4).norm();
            avg_len += (p2 - p3).norm();
            avg_len += (p2 - p4).norm();
            avg_len += (p3 - p4).norm();

            Eigen::Matrix3d E;
            E.col(0) = p1 - p4;
            E.col(1) = p2 - p4;
            E.col(2) = p3 - p4;

            tet_volume_signs[i] = E.determinant() >= 0.0 ? 1.0 : -1.0;
        }
        Eigen::MatrixXd posis = undeformed_mesh.toVertexMatrix();
        auto fix_posis = [&](int i1, int i2, int i3, int i4) -> volumetric_base::Tetrahedron {
            Eigen::Vector3d v1 = posis.row(i1);
            Eigen::Vector3d v2 = posis.row(i2);
            Eigen::Vector3d v3 = posis.row(i3);
            Eigen::Vector3d v4 = posis.row(i4);
            double vol = volumetric_base::geometry::volume_tetrahedron_signed(v1, v2, v3, v4);
            if (vol > 0) {
                return volumetric_base::Tetrahedron{i1, i2, i3, i4};
            }
            return volumetric_base::Tetrahedron{i1, i2, i4, i3}; // flip orientation
        };

        cellsize = avg_len / static_cast<double>(anz);
        for (const auto &prism: undeformed_mesh.getPrisms() ) {
            auto [i1,i2,i3,i4,i5,i6] = undeformed_mesh.getIndices(prism);
            const volumetric_base::Tetrahedron t1 = fix_posis(i1,i2,i3,i6);
            const volumetric_base::Tetrahedron t2 = fix_posis(i1,i3,i4,i6);
            const volumetric_base::Tetrahedron t3 = fix_posis(i3,i4,i5,i6);
            tets.emplace_back(t1);
            tets.emplace_back(t2);
            tets.emplace_back(t3);
        }

    }

    double signed_volume(const Eigen::Vector3d& a,
                         const Eigen::Vector3d& b,
                         const Eigen::Vector3d& c,
                         const Eigen::Vector3d& d) {
        return ((b - a).cross(c - a)).dot(d - a) / 6.0;
    }


bool CollisionDetection::check_exact_collision(
        const Eigen::Vector3d &position, int prism_idx,
        const Eigen::MatrixXd &posis, Eigen::Vector4d &bary_coordinates,
        Eigen::Vector4i &bary_indices) const {
    auto signed_volume = [](const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b,
                            const Eigen::Vector3d& c,
                            const Eigen::Vector3d& d) -> double {
        return ((b - a).cross(c - a)).dot(d - a) / 6.0;
    };
    //Potentially could have empty spots (boo) TODO
    const volumetric_base::Tetrahedron t1 = tets[prism_idx * 3];
    const volumetric_base::Tetrahedron t2 = tets[prism_idx * 3 + 1];
    const volumetric_base::Tetrahedron t3 = tets[prism_idx * 3 + 2];

    std::vector tets_local = { t1, t2, t3 };
    constexpr double eps = 1e-10;

    for (const auto& [i1,i2,i3,i4] : tets_local) {

        const Eigen::Vector3d& A = posis.row(i1);
        const Eigen::Vector3d& B = posis.row(i2);
        const Eigen::Vector3d& C = posis.row(i3);
        const Eigen::Vector3d& D = posis.row(i4);

        double V  = signed_volume(A, B, C, D);
        if (std::abs(V) < eps) continue;  // Degenerate

        double V1 = signed_volume(position, B, C, D);
        double V2 = signed_volume(A, position, C, D);
        double V3 = signed_volume(A, B, position, D);
        double V4 = signed_volume(A, B, C, position);

        double lambda1 = V1 / V;
        double lambda2 = V2 / V;
        double lambda3 = V3 / V;
        double lambda4 = V4 / V;

        bary_coordinates << lambda1, lambda2, lambda3, lambda4;

        if ((bary_coordinates.array() >= -eps).all()) {
            bary_indices << i1, i2, i3, i4;
            return true;
        }
    }

    return false;
}

std::vector<std::tuple<volumetric_base::LayerVertex, volumetric_base::Prism, Eigen::Vector4d, Eigen::Vector4i>>
CollisionDetection::test_for_prism_collision_candidates(
    const volumetric_base::LayeredMesh &mesh) {
        hash_grid.clear();
        for (const volumetric_base::LayerVertex &lv: mesh.getLayerVertices()) {
            auto [vv,layer] = lv;
            if (layer == volumetric_base::Layer::SKEL) continue;
            const Eigen::Vector3d v = mesh.position(lv) / cellsize;
            long int hash_ = hash(static_cast<int>(std::floor(v[0])), static_cast<int>(std::floor(v[1])),
                                  static_cast<int>(std::floor(v[2])));
            hash_grid[hash_].emplace_back(lv);
        }
        std::vector<std::tuple<volumetric_base::LayerVertex, volumetric_base::Prism, Eigen::Vector4d, Eigen::Vector4i>> collisions;
        const auto prisms = mesh.getPrisms();
        const Eigen::MatrixXd positions = mesh.toVertexMatrix();
        int num_threads = omp_get_max_threads();
        std::vector<std::vector<std::tuple<volumetric_base::LayerVertex, volumetric_base::Prism, Eigen::Vector4d, Eigen::Vector4i>> > thread_collisions(num_threads);

        #pragma omp parallel for
        for (int i = 0; i < static_cast<int> (prisms.size()); ++i) {
            const auto &prism = prisms[i];
            const auto & [j1,j2,j3,j4,j5,j6] = mesh.getIndices(prism);
            int tid=omp_get_thread_num();
            auto& local = thread_collisions[tid];
            Eigen::Matrix<double,3,6> points;
            points.col(0) = positions.row(j1);
            points.col(1) = positions.row(j2);
            points.col(2) = positions.row(j3);
            points.col(3) = positions.row(j4);
            points.col(4) = positions.row(j5);
            points.col(5) = positions.row(j6);

            const Eigen::Vector3d bb_min = points.rowwise().minCoeff();
            const Eigen::Vector3d bb_max = points.rowwise().maxCoeff();

            int bb_minI[3], bb_maxI[3];
            for (int j = 0; j < 3; ++j) {
                bb_minI[j] = static_cast<int>(floor(bb_min[j] / cellsize));
                bb_maxI[j] = static_cast<int>(ceil(bb_max[j] / cellsize));
            }
            for (int x = bb_minI[0]; x <= bb_maxI[0]; ++x) {
                for (int y = bb_minI[1]; y <= bb_maxI[1]; ++y) {
                    for (int z = bb_minI[2]; z <= bb_maxI[2]; ++z) {
                        auto it = hash_grid.find(hash(x, y, z));
                        if (it == hash_grid.end()) {
                            continue;
                        }
                        std::vector<volumetric_base::LayerVertex> vertices = hash_grid[hash(x, y, z)];
                        for (const volumetric_base::LayerVertex &v: vertices) {
                            int index = static_cast<int>(volumetric_base::LayeredMesh::Index(v));
                            //filter one ring neighborhood
                            if (index == j1 || index == j2 || index == j3 || index == j4 || index == j5 || index == j6 ) {
                                continue;
                            }
                            Eigen::Vector4d bary_coords;
                            Eigen::Vector4i bary_indices;
                            if (check_exact_collision(mesh.position(v), i,
                                                      positions, bary_coords,
                                                      bary_indices))
                                local.emplace_back(v,prism, bary_coords, bary_indices);

                        }
                    }
                }
            }
        }
        for (auto& local:thread_collisions) {
            collisions.insert(collisions.end(), local.begin(), local.end());
        }

        return collisions;
    }


    std::vector<std::tuple<pmp::Vertex, volumetric_base::Tetrahedron, volumetric_base::Prism> >
    CollisionDetection::test_for_collisions(const volumetric_base::LayeredMesh &mesh, pmp::SurfaceMesh &hires) const {
        std::pmr::unordered_map<long int, std::vector<pmp::Vertex> > hash_grid;
        for (const pmp::Vertex lv: hires.vertices()) {
            const Eigen::Vector3d v = hires.position(lv) / cellsize;
            long int hash_ = hash(static_cast<int>(floor(v[0])), static_cast<int>(floor(v[1])),
                                  static_cast<int>(floor(v[2])));
            hash_grid[hash_].emplace_back(lv);
        }

        std::vector<std::tuple<pmp::Vertex, volumetric_base::Tetrahedron, volumetric_base::Prism> > colliding_tets;


        const auto tets_and_cells = volumetric_base::tetrahedral::getTetrahedraAndCellsSafe(mesh);
        const Eigen::MatrixXd positions = mesh.toVertexMatrix();
        for (unsigned int i = 0; i < tets_and_cells.size(); ++i) {
            const auto &[tet,cell] = tets_and_cells[i];
            Eigen::PartialPivLU<Eigen::Matrix3d> lu;
            Eigen::Matrix3d T33;
            const auto &[i1,i2,i3,i4] = tet;
            T33.col(0) = positions.row(i1);
            T33.col(1) = positions.row(i2);
            T33.col(2) = positions.row(i3);
            const Eigen::Vector3d p4 = positions.row(i4);

            Eigen::Vector3d bb_min;
            Eigen::Vector3d bb_max;

            int bb_minI[3], bb_maxI[3];
            for (int j = 0; j < 3; ++j) {
                bb_min[j] = std::min(p4[j], T33.row(j).minCoeff());
                bb_max[j] = std::max(p4[j], T33.row(j).maxCoeff());

                bb_minI[j] = static_cast<int>(floor(bb_min[j] / cellsize));
                bb_maxI[j] = static_cast<int>(ceil(bb_max[j] / cellsize));
            }

            T33.col(0) -= p4;
            T33.col(1) -= p4;
            T33.col(2) -= p4;

            bool first = true;
            for (int x = bb_minI[0]; x < bb_maxI[0]; ++x) {
                for (int y = bb_minI[1]; y < bb_maxI[1]; ++y) {
                    for (int z = bb_minI[2]; z < bb_maxI[2]; ++z) {
                        auto it = hash_grid.find(hash(x, y, z));
                        if (it == hash_grid.end()) {
                            continue;
                        }
                        std::vector<pmp::Vertex> vertices = hash_grid[hash(x, y, z)];
                        for (const pmp::Vertex &v: vertices) {
                            if (first) {
                                lu.compute(T33);
                                first = false;
                            }
                            if (vertex_in_tet(lu, static_cast<Eigen::Vector3d>(hires.position(v)) - p4)) {
                                colliding_tets.emplace_back(v, tet, cell);
                            }
                        }
                    }
                }
            }
        }
        return colliding_tets;
    }

    std::vector<std::tuple<pmp::Vertex, std::array<int, 4> > > CollisionDetection::test_for_collisions(
        const pmp::SurfaceMesh &hires, std::vector<std::array<int, 4> > tets, const Eigen::MatrixXd &positions) const {

        std::pmr::unordered_map<long int, std::vector<pmp::Vertex> > hash_grid;
        for (const pmp::Vertex lv: hires.vertices()) {
            const Eigen::Vector3d v = hires.position(lv) / cellsize;
            long int hash_ = hash(static_cast<int>(floor(v[0])), static_cast<int>(floor(v[1])),
                                  static_cast<int>(floor(v[2])));
            hash_grid[hash_].emplace_back(lv);
        }
        std::vector<std::tuple<pmp::Vertex, std::array<int, 4> > > colliding_tets;
        for (unsigned int i = 0; i < tets.size(); ++i) {
            const auto &tet = tets[i];
            Eigen::PartialPivLU<Eigen::Matrix3d> lu;
            Eigen::Matrix3d T33;
            const auto &[i1,i2,i3,i4] = tet;
            T33.col(0) = positions.row(i1);
            T33.col(1) = positions.row(i2);
            T33.col(2) = positions.row(i3);
            const Eigen::Vector3d p4 = positions.row(i4);

            Eigen::Vector3d bb_min;
            Eigen::Vector3d bb_max;

            int bb_minI[3], bb_maxI[3];
            for (int j = 0; j < 3; ++j) {
                bb_min[j] = std::min(p4[j], T33.row(j).minCoeff());
                bb_max[j] = std::max(p4[j], T33.row(j).maxCoeff());

                bb_minI[j] = static_cast<int>(floor(bb_min[j] / cellsize));
                bb_maxI[j] = static_cast<int>(ceil(bb_max[j] / cellsize));
            }

            T33.col(0) -= p4;
            T33.col(1) -= p4;
            T33.col(2) -= p4;
            bool first = true;
            for (int x = bb_minI[0]; x <= bb_maxI[0]; ++x) {
                for (int y = bb_minI[1]; y <= bb_maxI[1]; ++y) {
                    for (int z = bb_minI[2]; z <= bb_maxI[2]; ++z) {
                        auto it = hash_grid.find(hash(x, y, z));
                        if (it == hash_grid.end()) {
                            continue;
                        }
                        std::vector<pmp::Vertex> vertices = hash_grid[hash(x, y, z)];
                        for (const pmp::Vertex &v: vertices) {

                            if (first) {
                                lu.compute(T33);
                                first = false;
                            }
                            if (vertex_in_tet(lu, static_cast<Eigen::Vector3d>(hires.position(v)) - p4)) {
                                colliding_tets.emplace_back(v, tet);
                            }
                        }
                    }
                }
            }
        }
        return colliding_tets;
    }
}
