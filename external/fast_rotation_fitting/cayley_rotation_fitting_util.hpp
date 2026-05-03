#pragma once
#include <Eigen/Core>


inline void addMatrixIntoCayleyRepresentation(std::vector<float> &vec, Eigen::Matrix3d &mat, int groupNum, int positionInGroup ){
    vec[groupNum*72+0*8+positionInGroup] = static_cast<float>(mat(0,0));
    vec[groupNum*72+1*8+positionInGroup] = static_cast<float>(mat(0,1));
    vec[groupNum*72+2*8+positionInGroup] = static_cast<float>(mat(0,2));
    vec[groupNum*72+3*8+positionInGroup] = static_cast<float>(mat(1,0));
    vec[groupNum*72+4*8+positionInGroup] = static_cast<float>(mat(1,1));
    vec[groupNum*72+5*8+positionInGroup] = static_cast<float>(mat(1,2));
    vec[groupNum*72+6*8+positionInGroup] = static_cast<float>(mat(2,0));
    vec[groupNum*72+7*8+positionInGroup] = static_cast<float>(mat(2,1));
    vec[groupNum*72+8*8+positionInGroup] = static_cast<float>(mat(2,2));

}

inline Eigen::Matrix3f MatrixFromCayleyRepresentation(std::vector<float> &vec, int groupNum, int positionInGroup ){
    Eigen::Matrix3f mat;
    mat(0,0) = vec[groupNum*72+0*8+positionInGroup];
    mat(0,1) = vec[groupNum*72+1*8+positionInGroup];
    mat(0,2) = vec[groupNum*72+2*8+positionInGroup];
    mat(1,0) = vec[groupNum*72+3*8+positionInGroup];
    mat(1,1) = vec[groupNum*72+4*8+positionInGroup];
    mat(1,2) = vec[groupNum*72+5*8+positionInGroup];
    mat(2,0) = vec[groupNum*72+6*8+positionInGroup];
    mat(2,1) = vec[groupNum*72+7*8+positionInGroup];
    mat(2,2) = vec[groupNum*72+8*8+positionInGroup];
    return mat;
}
