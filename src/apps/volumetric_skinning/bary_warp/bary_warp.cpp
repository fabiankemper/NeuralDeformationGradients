#include "bary_warp.h"
#include "pmp_ext/defs.h"

NAMESPACE_BEGIN(bary_warp)
  void BarycentricWarper::setup_warp_weights(const pmp::SurfaceMesh& input_mesh,
                                   const Eigen::SparseMatrix<double>& mapping,
                                   std::vector<WarpWeights>& warp_weights)
{
    warp_weights.clear();
    warp_weights.resize(mapping.rows());
    Eigen::SparseMatrix<double, Eigen::RowMajor> M = mapping;
        M.prune(1e-12);
    std::unordered_set<int> missing_verts;
    for (int i=0; i<M.rows(); ++i) {
        WarpWeights r{};
        double total_weight = 0.0;
        int k = 0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(M,i); it && k<32; ++it) {
            if (abs(it.value()) < DBL_MIN ) continue;
            r.indices[k] = it.col();
            r.weights[k] = it.value();
            total_weight += it.value();
            ++k;
        }
        if (total_weight == 0.0) {
            missing_verts.emplace(i);
        } else {
            for (int t=0; t<32; ++t) r.weights[t] /= total_weight; // zeros stay zero
        }
        warp_weights[i] = r;
    }

    // Gauss-Seidel fill (small set)
    for (int iter=0; iter<15; ++iter) {
        for (int vertex_id : missing_verts) {
            pmp::Vertex v(vertex_id);
            std::unordered_map<int,double> weights;
            weights.reserve(32);

            for (pmp::Vertex vv : input_mesh.vertices(v)) {
                const auto vv_weights = warp_weights[vv.idx()];
                for (int i=0; i<8; ++i) {
                    if (vv_weights.weights[i] == 0.0) break;
                    weights[vv_weights.indices[i]] += vv_weights.weights[i];
                }
            }
            warp_weights[v.idx()] = WarpWeights{};
            int j = 0;
            double sum = 0.0;
            for (const auto& kv : weights) {
                if (j >= 32) break;
                warp_weights[v.idx()].indices[j] = kv.first;
                warp_weights[v.idx()].weights[j] = kv.second;
                sum += kv.second;
                ++j;
            }
            if (sum > 0.0) {
                for (int t=0; t<j; ++t) warp_weights[v.idx()].weights[t] /= sum;
            }
        }
    }
}
NAMESPACE_END(bary_warp)