#pragma once
#include <random>

inline std::random_device rd{};
inline std::mt19937 generator{rd()};

NAMESPACE_BEGIN(random_util)

template<typename T>
void add_random_noise_to_matrix(T& mat, double mean, double sd)
{
    std::normal_distribution<> distribution{mean, sd};
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            double d = distribution(generator);
            mat(i, j) += d;
        }
    }
}

inline int getRandomInteger(int lower, int upper)
{
    std::uniform_int_distribution<> distr(lower, upper);
    return distr(generator);
}

template<typename Matrix>
std::tuple<double, double> get_mean_and_sd(const Matrix& mat)
{
    double mean = mat.mean();
    double variance = (mat.array() - mean).square().mean();
    double sd = sqrt(variance);
    return {mean, sd};
}

template<typename Matrix=Eigen::MatrixXd>
double get_average_absolute_deviation(const Matrix& mat)
{
    double mean = mat.mean();
    double average_abs_distance = (mat.array() - mean).abs().mean();
    return average_abs_distance;
}

NAMESPACE_END(random_util)
