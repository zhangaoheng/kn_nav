/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// ============================================================================
// 文件名: affine_constraint.h
// 用途:   仿射(线性)不等式约束容器, 约束形式为 l <= A*x <= u
//         (等式约束即 l == u); 用于把样条优化问题的约束整理后交给 QP 求解器
// 结构:   AffineConstraint 类, 内部保存约束矩阵与上下界向量
// 依赖:   common/base/type.h
// ============================================================================

#pragma once

#include <Eigen/Core>
#include <vector>

#include "common/base/type.h"

#define udrive_inf ((double)1e30)  // NOLINT

// 仿射约束容器: 描述 l <= A*x <= u, 支持追加多组约束
namespace common {
/**
 * @class AffineConstraint
 * @brief constraint in form of "l <= Ax <= u", equality constraint means l = u
 */
class AffineConstraint {
 public:
  AffineConstraint() = default;

  AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);

  const Eigen::MatrixXd& constraint_matrix() const;

  const std::vector<double>& lower_bound() const;

  const std::vector<double>& upper_bound() const;

// 追加一组约束(约束矩阵行数与上下界长度需一致)
  bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                     const std::vector<double>& lower_bound,
                     const std::vector<double>& upper_bound);

// 打印约束矩阵与上下界(调试用)
  void Print() const;

 private:
  Eigen::MatrixXd constraint_matrix_;

  std::vector<double> lower_bound_;

  std::vector<double> upper_bound_;
};
}  // namespace common
