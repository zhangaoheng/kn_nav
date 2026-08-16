// ============================================================================
// 文件：Exp_mat.h
// 用途：SO(3) 旋转群与李代数之间的映射：指数映射 Exp（罗德里格斯公式）与
//       对数映射 Log，是 ESKF 中旋转状态增量更新的核心数学工具。
// 结构：Exp 三个重载（旋转向量 / 角速度*dt / 三分量）+ Log 一个重载。
// 依赖：Eigen、opencv2/core（仅头文件引用）。
// 说明：上游开源算法（FAST-LIO），工作区内作为里程计前端使用；
//       与 so3_math.h 功能重叠，二者保留其一即可满足编译。
// ============================================================================
#ifndef EXP_MAT_H
#define EXP_MAT_H

#include <math.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
// #include <common_lib.h>

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

template<typename T>
// 指数映射：由旋转向量（角轴）求旋转矩阵，零向量返回单位阵
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T, typename Ts>
// 指数映射：由角速度与时间间隔 dt 求旋转增量矩阵（IMU 姿态递推用）
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T>
// 指数映射：角轴三分量版本（v1, v2, v3 即旋转向量分量）
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

// 对数映射：由旋转矩阵求旋转向量（小角度时用近似式 0.5*K 避免数值奇异）
/* Logrithm of a Rotation Matrix */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    T &&theta = std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

// template<typename T>
// cv::Mat Exp(const T &v1, const T &v2, const T &v3)
// {
    
//     T norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
//     cv::Mat Eye3 = cv::Mat::eye(3, 3, CV_32F);
//     if (norm > 0.0000001)
//     {
//         T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
//         cv::Mat K = (cv::Mat_<T>(3,3) << SKEW_SYM_MATRX(r_ang));

//         /// Roderigous Tranformation
//         return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
//     }
//     else
//     {
//         return Eye3;
//     }
// }

#endif
