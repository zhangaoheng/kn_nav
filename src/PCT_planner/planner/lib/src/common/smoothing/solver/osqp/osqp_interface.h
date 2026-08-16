// ============================================================================
// 文件名: osqp_interface.h
// 用途:   OSQP 二次规划求解器的薄封装接口: 把 Eigen 稀疏/稠密矩阵形式的
//         QP 问题 (min 0.5*x'Px + q'x, s.t. l <= Ax <= u) 转换为 osqp 的
//         CSC 稀疏格式后求解, 结果写回 x。属第三方求解器封装, 本文件只讲职责。
// 结构:   ColSparseMatrix (Eigen 列主序稀疏矩阵别名) + OsqpInterface 静态类
// 注意:   q/l/u 在求解过程中可能被 osqp 就地修改; 稠密接口内部会转稀疏。
// ============================================================================
#pragma once

#include <osqp/osqp.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace common {

using ColSparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;

/**
 * @class OsqpInterface (ADMM based qp solver)
 * @brief solve convex quadratic programming problem
 *
 * min_{x} 1/2 x'Px + q'x
 * s.t.    l <= Ax <= u
 *
 * Note: osqp use sparse matrix representation
 * [compressed-column](https://people.sc.fsu.edu/~jburkardt/data/cc/cc.html)
 */
// OSQP 求解接口: 全部为静态方法, 无内部状态; 底层为 ADMM 求解器,
// 输入输出均为 Eigen 类型, 矩阵格式转换在内部完成。
class OsqpInterface {
 public:
  // please notice that q, l, u may be modified during optimization
// 稀疏版求解: P/A 直接以 Eigen 稀疏矩阵传入, 内部转成 CSC 后求解。
  static bool Solve(const ColSparseMatrix& P,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
                    const ColSparseMatrix& A,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
                    Eigen::VectorXd* x);

  // Although dense matrix interface is provided, one should notice that dense
  // matrix P, A will be convert to sparse matrix anyway.
// 稠密版求解: P/A 为稠密矩阵, 内部会先转换为稀疏格式再求解,
// 调用更直观但多一次拷贝开销。
  static bool Solve(const Eigen::MatrixXd& P,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
                    const Eigen::MatrixXd& A,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
                    Eigen::VectorXd* x);
};

}  // namespace common