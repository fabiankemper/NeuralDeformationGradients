// =====================================================================================================================

#include "precompute_solver.h"
#include <pmp/exceptions.h>

// =====================================================================================================================

const Eigen::MatrixXd& PrecomputeSolver::solve(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::MatrixXd& B,
    const std::function<bool(unsigned int)>& is_constrained,
    const Eigen::MatrixXd& C
) {
#ifdef USE_MKL
    auto& iparm = solver.pardisoParameterArray();
    iparm[0]  = 1;   // defaults
    iparm[1]  = 2;   // METIS ordering
    iparm[2]  = 0;   // threads = MKL default
    iparm[7]  = 1;   // one iterative refinement (enough)
    iparm[24] = 1;   // parallel backsubstitution
    iparm[25] = 1;   // parallel factorization
    iparm[26] = 0;
#endif

    int n = 0;
    if (!precomputed) {
        idx.assign(A.cols(), -1);
        for (int i = 0; i < A.cols(); ++i) {
            if (!is_constrained(i)) idx[i] = n++;
        }
        n_unconstrained = n;

        BB.resize(n, B.cols());
        X.resize(B.rows(), B.cols());
        XX.resize(n, B.cols());

        // Preallocate our caching matrices
        C_float.resize(C.rows(), C.cols());
        rhs_penalty.resize(n, C.cols());
    }

    // Build float RHS (unconstrained rows only)
    OMP_PARALLEL_FOR()
    for (int i = 0; i < A.cols(); ++i) {
        if (idx[i] != -1) {
            BB.row(idx[i]) = B.row(i).cast<float>();
        }
    }

    if (precomputed) {
        OMP_PARALLEL_FOR()
        for(int i = 0; i < C.rows(); ++i) {
            C_float.row(i) = C.row(i).cast<float>();
        }
        rhs_penalty.noalias() = rhs_update * C_float;
        BB -= rhs_penalty;

    } else {

#ifdef USE_MKL
        std::vector<Eigen::Triplet<float, long long>> triplets;
#else
        std::vector<Eigen::Triplet<float, int>> triplets;
#endif
        triplets.reserve(A.nonZeros());

        for (int k = 0; k < A.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
                const int i = static_cast<int>(it.row());
                const int j = static_cast<int>(it.col());
                if (idx[i] != -1) {
                    if (idx[j] != -1) {
#ifdef USE_MKL
                        if (idx[i] <= idx[j]) {
                            triplets.emplace_back(
                                static_cast<MklIndex>(idx[i]),
                                static_cast<MklIndex>(idx[j]),
                                static_cast<float>(it.value())
                            );
                        }
#else
                        if (idx[i] <= idx[j]) {
                            triplets.emplace_back(idx[i], idx[j], static_cast<float>(it.value()));
                        }
#endif
                    }
                }
            }
        }

#ifdef USE_MKL
        Eigen::SparseMatrix<float, Eigen::ColMajor, MklIndex> AA(n, n);
        std::vector<Eigen::Triplet<float, MklIndex>> triplets_update;
#else
        Eigen::SparseMatrix<float, Eigen::ColMajor, int> AA(n, n);
        std::vector<Eigen::Triplet<float, int>> triplets_update;
#endif

        AA.setFromTriplets(triplets.begin(), triplets.end());
        AA.makeCompressed();

        solver.compute(AA);
        if (solver.info() != Eigen::Success) {
            throw pmp::SolverException(std::string{__func__} + ": Failed to factorize linear system.");
        }

        rhs_update.resize(n_unconstrained, C.rows());

        for (int k = 0; k < A.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
                const int i = static_cast<int>(it.row());
                const int j = static_cast<int>(it.col());
                if (idx[i] != -1 && idx[j] == -1) {
#ifdef USE_MKL
                    triplets_update.emplace_back(
                        static_cast<MklIndex>(idx[i]),
                        static_cast<MklIndex>(j),
                        static_cast<float>(it.value())
                    );
#else
                    triplets_update.emplace_back(idx[i], j, static_cast<float>(it.value()));
#endif
                }
            }
        }
        rhs_update.setFromTriplets(triplets_update.begin(), triplets_update.end());
        OMP_PARALLEL_FOR()
        for(int i = 0; i < C.rows(); ++i) {
            C_float.row(i) = C.row(i).cast<float>();
        }
        rhs_penalty.noalias() = rhs_update * C_float;
        BB -= rhs_penalty;

        precomputed = true;
    }
    XX.noalias() = solver.solve(BB);
    if (solver.info() != Eigen::Success) {
        throw pmp::SolverException(std::string{__func__} + ": Failed to solve linear system.");
    }
    OMP_PARALLEL_FOR()
    for (int i = 0; i < A.cols(); ++i) {
        if (idx[i] == -1) {
            X.row(i) = C.row(i);
        } else {
            X.row(i) = XX.row(idx[i]).cast<double>();
        }
    }

    return X;
}