#include <robot_controller/Util/Util.hpp>

namespace rc
{
    template Eigen::Matrix<float, Eigen::Dynamic, 1> ToEigen<float>(std::vector<float> data);

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data)
    {
        Eigen::Matrix<T, Eigen::Dynamic, 1> data_eigen;
        size_t n = data.size();
        data_eigen.resize(n, 1);
        for (size_t i = 0; i < n; ++i)
        {
            data_eigen(i) = data.at(i);
        }
        return data_eigen;
    }

    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        p(0) = mat(0, 3);
        p(1) = mat(1, 3);
        p(2) = mat(2, 3);

        Eigen::Vector3f rot = mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        p(5) = rot(0);
        p(4) = rot(1);
        p(3) = rot(2);
    }

    float PoseMatToTransAngRad(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        p.head(3) = mat.block<3, 1>(0, 3);

        Eigen::AngleAxisf angular_d{mat.block<3, 3>(0, 0)};

        p(3) = angular_d.axis()(0) * angular_d.angle();
        p(4) = angular_d.axis()(1) * angular_d.angle();
        p(5) = angular_d.axis()(2) * angular_d.angle();

        return angular_d.angle();
    }

    int64_t GetTickUs()
    {
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);

        return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
    }

}
