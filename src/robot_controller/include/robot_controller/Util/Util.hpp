#pragma once

#include <robot_controller/common.hpp>

namespace rc
{
    typedef Eigen::Matrix<float, 6, 1> Vector6f;

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data);

    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, Vector6f& p);
    float PoseMatToTransAngRad(const Eigen::Matrix4f& mat, Vector6f& p);

    int64_t GetTickUs();

}
