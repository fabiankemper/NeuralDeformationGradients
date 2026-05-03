#pragma once

int fit_rotation_small_no_avx(
    float* Mf,
    float* Rf,
    int num_of_group);

#ifdef HAVE_AVX2
int fit_rotation_small_avx(
    float* Mf,
    float* Rf,
    int num_of_group);
#endif // #ifdef HAVE_AVX2

int fit_rotation_no_avx(float* Mf, float* Rf, int num_of_group, int max_iter);
int fit_rotation_no_avx(
    float* Mf,
    float* Rf,
    int num_of_group);


#ifdef HAVE_AVX2
int fit_rotation_avx(
    float* Mf,
    float* Rf,
    int num_of_group);
#endif // #ifdef HAVE_AVX2
