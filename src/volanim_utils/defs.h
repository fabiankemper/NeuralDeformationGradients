#ifndef VOLANIM_UTILS_DEFS_H
#define VOLANIM_UTILS_DEFS_H

// --- namespaces
#if !defined(NAMESPACE_BEGIN)
#  define NAMESPACE_BEGIN(name) namespace name {
#endif

#if !defined(NAMESPACE_END)
#  define NAMESPACE_END(name) }
#endif

//// to underlaying
//#if (__cplusplus < 202302L)
//
//template <typename E>
//constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
//    return static_cast<typename std::underlying_type<E>::type>(e);
//}
//
//#endif

// --- OpenMP statements
// disable openMP for macOS

// parallel for
#if defined(NO_USE_OPENMP)
#define OMP_PARALLEL_FOR(...)
#else
#define OMP_STRINGIFY(x) #x
#define OMP_PARALLEL_FOR(...) _Pragma(OMP_STRINGIFY(omp parallel for __VA_ARGS__))
#endif

// atomic
#if defined(NO_USE_OPENMP)
#define OMP_ATOMIC
#else
#define OMP_ATOMIC _Pragma("omp atomic")
#endif




#endif //VOLANIM_UTILS_DEFS_H
